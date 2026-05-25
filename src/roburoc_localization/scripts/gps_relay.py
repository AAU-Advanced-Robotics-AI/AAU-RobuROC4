#!/usr/bin/env python3
"""
gps_relay.py -- Relay GPS as body-frame odometry inputs for the local and global EKFs.

For the global EKF (/odometry/gps): receives the raw antenna ENU position from
gps_to_enu, applies the lever-arm correction to obtain the base_link position,
and publishes to /odometry/gps.  Only published on RTK-quality fixes after the
Kabsch transform is available.

  heading_ENU = yaw_odom (from LIO) + theta_kabsch (from lio_to_enu)
  base_link_ENU = antenna_ENU - R(heading_ENU) @ [lever_arm_x, lever_arm_y]

For the local EKF (/odometry/gps/local): receives ENU velocity from the u-blox
driver and rotates it into the robot body frame using the same heading.

Publications
------------
  /odometry/gps         nav_msgs/Odometry
                        frame_id=map  child_frame_id=base_link
                        pose = lever-arm corrected position (RTK quality only)
                        Only published when the Kabsch transform is available.

  /odometry/gps/local   nav_msgs/Odometry
                        frame_id=odom  child_frame_id=base_link
                        twist = body-frame velocity  (vx=forward, vy=left)
                        pose  = sentinel covariance  (EKF ignores position)
                        Only published when the Kabsch transform is available.

Subscriptions
-------------
  /odometry/gps/raw              nav_msgs/Odometry  (raw antenna ENU, from gps_to_enu)
  /ublox_gps_node/fix_velocity   geometry_msgs/TwistWithCovarianceStamped (ENU)
  /odometry/lio                  nav_msgs/Odometry  (for current LIO yaw)
  /localization/lio_to_enu       nav_msgs/Odometry  (for Kabsch theta, transient_local)

Parameters
----------
  lever_arm_x   float  -0.4775  GPS antenna X offset in base_link [m]
  lever_arm_y   float   0.285   GPS antenna Y offset in base_link [m]
  rtk_cov_max   float  0.01    Max E/N variance to accept a GPS fix [m^2]
  vel_max_age   float  0.5     Max age of fix_velocity [s]; drop if older
  sentinel_cov  float  1e6    Position covariance sentinel [m^2]
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class GpsRelay(Node):

    def __init__(self):
        super().__init__("gps_relay")

        self.declare_parameter("lever_arm_x",  -0.4775)
        self.declare_parameter("lever_arm_y",   0.285)
        self.declare_parameter("rtk_cov_max",   0.01)
        self.declare_parameter("vel_max_age",   0.5)
        self.declare_parameter("sentinel_cov",  1e6)

        self._lx          = self.get_parameter("lever_arm_x").value
        self._ly          = self.get_parameter("lever_arm_y").value
        self._rtk_cov_max = self.get_parameter("rtk_cov_max").value
        self._vel_max_age = self.get_parameter("vel_max_age").value
        self._sentinel    = self.get_parameter("sentinel_cov").value

        self._last_vel:     TwistWithCovarianceStamped | None = None
        self._yaw_odom:     float = 0.0
        self._theta_kabsch: float | None = None   # odom->ENU rotation angle

        latch_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            Odometry, "/odometry/gps/raw", self._gps_raw_cb, 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/ublox_gps_node/fix_velocity", self._vel_cb, 10)
        self.create_subscription(
            Odometry, "/odometry/lio", self._lio_cb, 10)
        self.create_subscription(
            Odometry, "/localization/lio_to_enu", self._kabsch_cb, latch_qos)

        self._pub_gps   = self.create_publisher(Odometry, "/odometry/gps",       10)
        self._pub_local = self.create_publisher(Odometry, "/odometry/gps/local", 10)

        self.get_logger().info(
            f"gps_relay: lever_arm=({self._lx:.4f}, {self._ly:.4f}) m  "
            f"rtk_cov_max={self._rtk_cov_max} m2  "
            "waiting for Kabsch transform from /localization/lio_to_enu ...")

    def _vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        self._last_vel = msg
        self._try_publish()

    def _lio_cb(self, msg: Odometry) -> None:
        self._yaw_odom = _yaw_from_quat(msg.pose.pose.orientation)

    def _kabsch_cb(self, msg: Odometry) -> None:
        self._theta_kabsch = _yaw_from_quat(msg.pose.pose.orientation)
        self.get_logger().info(
            f"gps_relay: Kabsch theta received -- "
            f"theta={math.degrees(self._theta_kabsch):.2f} deg")

    def _gps_raw_cb(self, msg: Odometry) -> None:
        """Apply lever-arm correction and publish /odometry/gps."""
        if self._theta_kabsch is None:
            return

        # RTK quality check: gps_to_enu sets sentinel covariance for bad fixes
        cov_e = msg.pose.covariance[0]
        cov_n = msg.pose.covariance[7]
        if cov_e > self._rtk_cov_max or cov_n > self._rtk_cov_max:
            return

        # Lever-arm correction: base_link = antenna - R(yaw_ENU) @ [lx, ly]
        yaw_enu = self._yaw_odom + self._theta_kabsch
        c = math.cos(yaw_enu)
        s = math.sin(yaw_enu)
        lever_e = c * self._lx - s * self._ly
        lever_n = s * self._lx + c * self._ly

        odom = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = msg.pose.pose.position.x - lever_e
        odom.pose.pose.position.y = msg.pose.pose.position.y - lever_n
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0]  = cov_e    # x-x (east)
        odom.pose.covariance[7]  = cov_n    # y-y (north)
        odom.pose.covariance[14] = 0.01     # z  (not estimated)
        for i in (21, 28, 35):              # roll, pitch, yaw — not from GPS
            odom.pose.covariance[i] = self._sentinel
        for i in range(36):                 # no twist from GPS position fix
            odom.twist.covariance[i] = self._sentinel if (i % 7 == 0) else 0.0
        self._pub_gps.publish(odom)

    def _try_publish(self) -> None:
        if self._last_vel is None or self._theta_kabsch is None:
            return

        # Check velocity age
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        vel_sec = (self._last_vel.header.stamp.sec
                   + self._last_vel.header.stamp.nanosec * 1e-9)
        if abs(now_sec - vel_sec) > self._vel_max_age:
            return

        # Robot heading in ENU frame
        heading = self._yaw_odom + self._theta_kabsch
        c = math.cos(heading)
        s = math.sin(heading)

        # ENU -> body: R_body_ENU = [[c, s], [-s, c]]
        # v_forward = v_east*c + v_north*s
        # v_left    = -v_east*s + v_north*c
        lv = self._last_vel.twist.twist.linear
        v_east, v_north = lv.x, lv.y
        v_fwd  =  v_east * c + v_north * s
        v_left = -v_east * s + v_north * c

        # Rotate 2x2 ENU velocity covariance to body frame
        # C_body = R_body_ENU @ C_ENU @ R_body_ENU^T
        cov = self._last_vel.twist.covariance
        c_enu = np.array([[cov[0], cov[1]], [cov[6], cov[7]]])
        R = np.array([[c, s], [-s, c]])
        c_body = R @ c_enu @ R.T

        odom = Odometry()
        odom.header.stamp    = self._last_vel.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        # Sentinel pose covariance so EKF ignores position from this source
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = self._sentinel

        # Body-frame velocity
        odom.twist.twist.linear.x = v_fwd
        odom.twist.twist.linear.y = v_left
        odom.twist.twist.linear.z = 0.0
        odom.twist.covariance[0]  = float(c_body[0, 0])   # vx-vx
        odom.twist.covariance[1]  = float(c_body[0, 1])   # vx-vy
        odom.twist.covariance[6]  = float(c_body[1, 0])   # vy-vx
        odom.twist.covariance[7]  = float(c_body[1, 1])   # vy-vy
        odom.twist.covariance[35] = self._sentinel          # angular vel not provided

        self._pub_local.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GpsRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
