#!/usr/bin/env python3
"""
gps_to_enu.py — Convert GPS NavSatFix to local cartesian Odometry aligned to
                 the FAST_LIO odom frame, expressed at base_link.

Replaces navsat_transform in the offline bag-processing pipeline.

Subscriptions
-------------
  /ublox_gps_node/fix          (sensor_msgs/NavSatFix)
  /ublox_gps_node/fix_velocity (geometry_msgs/TwistWithCovarianceStamped, ENU)
  /Odometry                    (nav_msgs/Odometry, FAST_LIO — for current yaw)

Publication
-----------
  /odometry/gps  (nav_msgs/Odometry, frame_id=odom, child_frame_id=base_link)

Algorithm
---------
  1. Origin: first GPS fix with status ≥ 0 sets (lat0, lon0).  The datum is
     the GPS antenna position at that moment.  Because FAST_LIO's odom origin is
     base_link (not the antenna), ENU (0,0) is offset from odom (0,0) by the
     lever arm (lx, ly).  This is corrected in step 3b so that the published
     track starts at base_link = odom (0,0) at t=0.
  2. Flat-Earth ENU: east = Δlon·cos(lat0)·R, north = Δlat·R  (valid ≤ 10 km).
  3. Frame alignment — ENU → odom:
       FAST_LIO's odom frame has  x = robot-forward-at-init,  y = robot-left-at-init.
       Once the robot exceeds `speed_threshold` m/s, `bearing_samples` GPS velocity
       bearing readings are averaged (circular mean) to lock the rotation angle H.
           x_odom_antenna = east·sin(H) + north·cos(H)
           y_odom_antenna = -east·cos(H) + north·sin(H)
  4. Lever-arm correction — antenna → base_link:
       The GPS antenna is mounted at (lx, ly) in the base_link frame (from URDF).
       Using the current robot yaw ψ from /Odometry:
           x_base = x_odom_antenna - (lx·cos ψ - ly·sin ψ)
           y_base = y_odom_antenna - (lx·sin ψ + ly·cos ψ)
       If no /Odometry yaw is available yet, the last known yaw (or 0.0) is used.
  5. Fallback: if `bearing_timeout` seconds elapse without the robot moving, publish
     raw ENU at the antenna position (frame_id='gps_enu') so there is always output.

Parameters
----------
  speed_threshold   (float, default 0.3):  Min GPS speed (m/s) to start bearing estimation.
  bearing_samples   (int,   default 5):    Number of velocity samples to average.
  bearing_timeout   (float, default 15.0): Wall seconds to wait before falling back to ENU.
  lever_arm_x       (float, default -0.428): GPS antenna X in base_link frame (from URDF).
  lever_arm_y       (float, default  0.295): GPS antenna Y in base_link frame (from URDF).

Usage
-----
  Launched automatically by bag_process.launch.py.
  To run standalone:
    ros2 run roburoc_localization gps_to_enu.py
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# WGS-84 equatorial radius (metres)
_R_EARTH = 6_378_137.0


class GpsToEnu(Node):

    def __init__(self):
        super().__init__('gps_to_enu')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('speed_threshold', 0.3)
        self.declare_parameter('bearing_samples', 5)
        self.declare_parameter('bearing_timeout', 15.0)
        # Lever arm: GPS antenna position expressed in base_link frame (from URDF).
        # Update these if the antenna is remounted.
        self.declare_parameter('lever_arm_x', -0.428)  # metres forward of base_link
        self.declare_parameter('lever_arm_y',  0.295)  # metres left   of base_link

        self._speed_thr  = self.get_parameter('speed_threshold').value
        self._n_samples  = int(self.get_parameter('bearing_samples').value)
        self._timeout    = self.get_parameter('bearing_timeout').value
        self._lx         = self.get_parameter('lever_arm_x').value
        self._ly         = self.get_parameter('lever_arm_y').value

        # ── State ─────────────────────────────────────────────────────────────
        self._lat0: float | None = None   # geodetic datum
        self._lon0: float | None = None

        # Bearing alignment (radians, compass: 0=North, +π/2=East)
        self._bearing: float | None = None
        self._bearing_buf: list[float] = []
        self._fallen_back = False

        # Wall-clock time at which datum was first set (for timeout)
        self._datum_wall_sec: float | None = None

        # Current robot yaw in odom frame, updated from /Odometry.
        # Defaults to 0.0 (used before first /Odometry message arrives).
        self._yaw: float = 0.0

        # ── I/O ───────────────────────────────────────────────────────────────
        self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self._fix_cb,
            10,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/ublox_gps_node/fix_velocity',
            self._vel_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            '/Odometry',
            self._odom_cb,
            10,
        )
        self._pub = self.create_publisher(Odometry, '/odometry/gps', 10)

        # Timeout watchdog — fires at wall-clock rate regardless of sim time
        self._watchdog = self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            f'gps_to_enu: speed_threshold={self._speed_thr} m/s, '
            f'bearing_samples={self._n_samples}, '
            f'bearing_timeout={self._timeout} s, '
            f'lever_arm=({self._lx:.3f}, {self._ly:.3f}) m'
        )

    # ── Odometry callback: track current yaw ───────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        # Extract yaw from the quaternion (only z and w matter for a yaw-only rotation)
        q = msg.pose.pose.orientation
        # yaw = atan2(2(wz + xy), 1 - 2(yy + zz))  — full formula for robustness
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    # ── Velocity callback: bearing estimation ────────────────────────────────

    def _vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        if self._bearing is not None:
            return                              # already locked

        v_east  = msg.twist.twist.linear.x     # ENU East
        v_north = msg.twist.twist.linear.y     # ENU North
        speed   = math.hypot(v_east, v_north)

        if speed < self._speed_thr:
            return

        bearing = math.atan2(v_east, v_north)  # compass (0=N, +π/2=E)
        self._bearing_buf.append(bearing)

        if len(self._bearing_buf) >= self._n_samples:
            # Circular mean to avoid wrap-around artefacts near ±π
            sin_sum = sum(math.sin(b) for b in self._bearing_buf)
            cos_sum = sum(math.cos(b) for b in self._bearing_buf)
            self._bearing = math.atan2(sin_sum, cos_sum)
            self.get_logger().info(
                f'gps_to_enu: bearing locked from GPS velocity → '
                f'{math.degrees(self._bearing):.1f}° (0=N, +90=E). '
                f'Output frame: odom (aligned to FAST_LIO).'
            )
            self._watchdog.cancel()

    # ── Fix callback: convert and publish ────────────────────────────────────

    def _fix_cb(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:              # NavSatStatus.STATUS_NO_FIX = -1
            return

        lat, lon = msg.latitude, msg.longitude

        # Set datum on first valid fix
        if self._lat0 is None:
            self._lat0 = lat
            self._lon0 = lon
            self._datum_wall_sec = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'gps_to_enu: datum set → lat={lat:.6f}°, lon={lon:.6f}°'
            )
            return                             # skip origin itself (0,0)

        if self._bearing is None and not self._fallen_back:
            return                             # waiting for bearing lock or timeout

        # Flat-Earth ENU (valid for distances << 100 km from datum)
        east  = math.radians(lon - self._lon0) * math.cos(math.radians(self._lat0)) * _R_EARTH
        north = math.radians(lat - self._lat0) * _R_EARTH

        if self._fallen_back:
            # No bearing available — publish raw ENU at antenna position (no correction)
            x, y  = east, north
            frame = 'gps_enu'
        else:
            # Step 3 — Rotate ENU → FAST_LIO odom frame using initial compass bearing H:
            #   FAST_LIO x (forward) = [sin H, cos H] in ENU
            #   FAST_LIO y (left)    = [-cos H, sin H] in ENU
            H  = self._bearing
            xa =  east * math.sin(H) + north * math.cos(H)   # antenna in odom
            ya = -east * math.cos(H) + north * math.sin(H)

            # Origin correction: the ENU datum was set at the GPS antenna position,
            # but FAST_LIO's odom origin is base_link.  At t=0 (yaw=0) the antenna
            # sits at (lx, ly) in odom, so ENU (0,0) should map to odom (lx, ly),
            # not (0,0).  Adding (lx, ly) here fixes the constant datum offset so
            # that the GPS track is anchored to base_link at t=0.
            xa += self._lx
            ya += self._ly

            # Step 4 — Lever-arm correction: shift antenna → base_link
            # The antenna offset (lx, ly) in base_link is rotated by current yaw ψ
            # into the odom frame, then subtracted:
            #   p_base = p_antenna - R(ψ) * [lx, ly]
            psi  = self._yaw
            x    = xa - (self._lx * math.cos(psi) - self._ly * math.sin(psi))
            y    = ya - (self._lx * math.sin(psi) + self._ly * math.cos(psi))
            frame = 'odom'

        odom                      = Odometry()
        odom.header.stamp         = msg.header.stamp
        odom.header.frame_id      = frame
        odom.child_frame_id       = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        # Horizontal position covariance from NavSatFix (row-major 3×3 → 6×6 slots)
        odom.pose.covariance[0]  = msg.position_covariance[0]   # xx ≈ east var
        odom.pose.covariance[7]  = msg.position_covariance[4]   # yy ≈ north var
        odom.pose.covariance[14] = 0.01                         # zz fixed

        self._pub.publish(odom)

    # ── Timeout watchdog ──────────────────────────────────────────────────────

    def _watchdog_cb(self) -> None:
        if self._datum_wall_sec is None:
            return                             # datum not set yet

        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._datum_wall_sec
        if elapsed < self._timeout:
            return

        self._watchdog.cancel()
        self._fallen_back = True
        self.get_logger().warn(
            f'gps_to_enu: no motion detected after {self._timeout:.0f} s — '
            f'falling back to raw ENU output (x=East, y=North, frame_id=gps_enu). '
            f'FAST_LIO and GPS tracks will be rotated relative to each other by the '
            f'robot\'s initial heading.'
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GpsToEnu())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
