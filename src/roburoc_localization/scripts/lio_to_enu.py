#!/usr/bin/env python3
"""
lio_to_enu.py -- Online GPS/LIO Kabsch alignment for the live localization stack.

Subscribes to the RTK-GPS antenna positions in ENU (datum from gps_to_enu) and
to FAST-LIO2 odometry, and continuously estimates the 2-D rigid transform
T: odom -> map (ENU) using an online sliding-window Kabsch alignment.

The estimated transform is published as nav_msgs/Odometry on
/localization/lio_to_enu (transient_local QoS):
  header.frame_id = "map"
  child_frame_id  = "odom"
  pose.pose       = T: odom origin expressed in map (ENU)
  pose.covariance = combined alignment uncertainty (GPS RTK + Kabsch residual)

No TF is published.  The EKF (ekf_global) owns the map->odom TF by consuming
/odometry/lio/global (from lio_relay.py, which applies this transform) and
/odometry/gps (from gps_relay.py, which applies the lever-arm correction).

Publications
------------
  /localization/lio_to_enu   nav_msgs/Odometry  (transient_local)

Subscriptions
-------------
  /localization/datum              sensor_msgs/NavSatFix  (transient_local)
  /ublox_gps_node/fix              sensor_msgs/NavSatFix
  /ublox_gps_node/fix_velocity     geometry_msgs/TwistWithCovarianceStamped
  <lio_topic>                      nav_msgs/Odometry  (default /odometry/lio)

Parameters
----------
  calib_min_baseline  float   8.0     Min GPS path for initial alignment [m]
  rtk_cov_max         float   0.01    Max E/N variance for RTK quality [m^2]
  speed_threshold     float   0.20    Min speed to accept a GPS fix [m/s]
  lever_arm_x         float  -0.4775  GPS antenna X offset in base_link [m]
  lever_arm_y         float   0.285   GPS antenna Y offset in base_link [m]
  calib_timeout       float  90.0     Warn if alignment not achieved after N s
  drift_update_dist   float  10.0     Re-solve Kabsch every N metres [m]
  drift_window_dist   float  50.0     Spatial extent of sliding window [m]
  min_window_pairs    int     5       Min pairs required to trigger re-solve
  gps_lost_timeout    float   2.0     Declare GPS lost after N s without RTK [s]
  lio_topic           str    /odometry/lio
  vel_max_age         float   0.05    Max age of cached GPS velocity [s]
  lio_buf_size        int    200      LIO pose buffer depth for interpolation
"""

import collections
import enum
import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

_R_EARTH = 6_378_137.0


# ===========================================================================
# Kabsch / geometry helpers
# ===========================================================================

def _kabsch_2d(P: np.ndarray, Q: np.ndarray,
               weights: Optional[np.ndarray] = None
               ) -> Tuple[np.ndarray, np.ndarray, float]:
    """Weighted 2-D least-squares rigid transform: Q ~= R @ P + t."""
    if weights is None:
        weights = np.ones(len(P), dtype=np.float64)
    w = weights / weights.sum()
    p_bar = (w[:, None] * P).sum(axis=0)
    q_bar = (w[:, None] * Q).sum(axis=0)
    H = ((P - p_bar) * w[:, None]).T @ (Q - q_bar)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = q_bar - R @ p_bar
    return R, t, math.atan2(R[1, 0], R[0, 0])


def _kabsch_rms(P: np.ndarray, Q: np.ndarray,
                R: np.ndarray, t: np.ndarray,
                weights: np.ndarray) -> float:
    aligned = (R @ P.T).T + t
    res = np.linalg.norm(aligned - Q, axis=1)
    w = weights / weights.sum()
    return float(math.sqrt((w * res ** 2).sum()))


def _rot2d(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _theta_to_quat(theta: float) -> Tuple[float, float, float, float]:
    h = theta * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)


class _State(enum.Enum):
    WAITING_DATUM = "WAITING_DATUM"
    ALIGNING      = "ALIGNING"
    ALIGNED       = "ALIGNED"


# ===========================================================================
# Node
# ===========================================================================

class LioToEnu(Node):

    def __init__(self):
        super().__init__("lio_to_enu")

        # -- Parameters -------------------------------------------------------
        self.declare_parameter("calib_min_baseline",  5.0)
        self.declare_parameter("rtk_cov_max",         0.05)
        self.declare_parameter("speed_threshold",     0.20)
        self.declare_parameter("lever_arm_x",        -0.4775)
        self.declare_parameter("lever_arm_y",         0.285)
        self.declare_parameter("calib_timeout",      90.0)
        self.declare_parameter("drift_update_dist",  2.0)
        self.declare_parameter("drift_window_dist",  20.0)
        self.declare_parameter("min_window_pairs",    5)
        self.declare_parameter("gps_lost_timeout",    2.0)
        self.declare_parameter("lio_topic",          "/odometry/lio")
        self.declare_parameter("vel_max_age",         0.5)
        self.declare_parameter("lio_buf_size",        200)

        self._calib_min      = self.get_parameter("calib_min_baseline").value
        self._rtk_cov_max    = self.get_parameter("rtk_cov_max").value
        self._speed_thr      = self.get_parameter("speed_threshold").value
        self._lx             = self.get_parameter("lever_arm_x").value
        self._ly             = self.get_parameter("lever_arm_y").value
        self._calib_timeout  = self.get_parameter("calib_timeout").value
        self._drift_update_d = self.get_parameter("drift_update_dist").value
        self._drift_win_dist = self.get_parameter("drift_window_dist").value
        self._min_win_pairs  = int(self.get_parameter("min_window_pairs").value)
        self._gps_lost_to    = self.get_parameter("gps_lost_timeout").value
        self._vel_max_age    = self.get_parameter("vel_max_age").value
        self._lio_buf_size   = int(self.get_parameter("lio_buf_size").value)
        lio_topic            = self.get_parameter("lio_topic").value

        # -- Datum (set via /localization/datum subscription) -----------------
        self._lat0: Optional[float] = None
        self._lon0: Optional[float] = None
        self._state = _State.WAITING_DATUM
        self._datum_wall_sec: Optional[float] = None

        # -- GPS quality tracking ---------------------------------------------
        self._gps_ok: bool = False
        self._last_rtk_wall: Optional[float] = None   # header stamp of last rtk_ok+moving fix
        self._last_recv_stamp: Optional[float] = None  # header stamp of last received fix (any quality)

        # -- LIO pose buffer --------------------------------------------------
        # Entries: (stamp_sec, x, y, psi)
        self._lio_buf: collections.deque = collections.deque(maxlen=self._lio_buf_size)

        # -- Calibration buffer (ALIGNING phase) ------------------------------
        # Entries: dict with keys lio_ant, gps_enu, weight, cov_e, cov_n
        self._calib_buf: list = []
        self._calib_path: float = 0.0
        self._calib_last_gps: Optional[np.ndarray] = None

        # -- Sliding window (ALIGNED phase) -----------------------------------
        # Entries: (lio_ant: np.ndarray, gps_enu: np.ndarray, weight: float,
        #           cov_e: float, cov_n: float)
        self._win: collections.deque = collections.deque()
        self._win_path: float = 0.0
        self._win_last_gps: Optional[np.ndarray] = None
        self._drift_since_solve: float = 0.0

        # -- Current best transform -------------------------------------------
        self._R: Optional[np.ndarray] = None
        self._t: Optional[np.ndarray] = None
        self._theta: float = 0.0

        # -- Cached GPS velocity ----------------------------------------------
        self._last_vel: Optional[TwistWithCovarianceStamped] = None

        # -- Publishers / subscribers -----------------------------------------
        latch_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            NavSatFix, "/localization/datum",     self._datum_cb, latch_qos)
        self.create_subscription(
            NavSatFix, "/ublox_gps_node/fix",     self._fix_cb,   10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/ublox_gps_node/fix_velocity",       self._vel_cb,   10)
        self.create_subscription(
            Odometry,  lio_topic,                 self._lio_cb,   10)

        self._pub_transform = self.create_publisher(
            Odometry, "/localization/lio_to_enu", latch_qos)

        self.create_timer(1.0, self._watchdog_cb)
        self.create_timer(1.0, self._gps_loss_cb)

        self.get_logger().info(
            f"lio_to_enu: calib_min={self._calib_min} m  "
            f"rtk_cov_max={self._rtk_cov_max} m2  "
            f"speed_thr={self._speed_thr} m/s  "
            f"lever_arm=({self._lx:.4f}, {self._ly:.4f}) m  "
            f"drift_win={self._drift_win_dist} m")

    # =========================================================================
    # Subscriber callbacks
    # =========================================================================

    def _datum_cb(self, msg: NavSatFix) -> None:
        # Once aligned the datum is locked; ignore further updates.
        if self._state == _State.ALIGNED:
            return
        # Ignore duplicates (transient_local re-delivery of the same message).
        if (self._lat0 is not None
                and abs(msg.latitude  - self._lat0) < 1e-9
                and abs(msg.longitude - self._lon0) < 1e-9):
            return

        prev_lat, prev_lon = self._lat0, self._lon0
        self._lat0 = msg.latitude
        self._lon0 = msg.longitude
        self._state = _State.ALIGNING
        self._datum_wall_sec = self.get_clock().now().nanoseconds * 1e-9

        if prev_lat is not None:
            # Stale transient_local datum was replaced — clear the buffer so
            # pairs collected against the wrong ENU origin are discarded.
            self._calib_buf.clear()
            self._calib_path      = 0.0
            self._calib_last_gps  = None
            self.get_logger().warn(
                f"lio_to_enu: datum updated (stale datum replaced) -- "
                f"old=({prev_lat:.6f}, {prev_lon:.6f})  "
                f"new=({self._lat0:.6f}, {self._lon0:.6f})  "
                f"Calibration buffer cleared.")
        else:
            self.get_logger().info(
                f"lio_to_enu: datum received -- lat={self._lat0:.6f} lon={self._lon0:.6f}  "
                f"Collecting trajectory for initial alignment "
                f"(need {self._calib_min:.1f} m of RTK-quality motion)")

    def _lio_cb(self, msg: Odometry) -> None:
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._lio_buf.append((
            stamp_sec,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _yaw_from_quat(msg.pose.pose.orientation),
        ))

    def _vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        self._last_vel = msg

    def _fix_cb(self, msg: NavSatFix) -> None:
        if self._state == _State.WAITING_DATUM or self._lat0 is None:
            return
        if msg.status.status < 0:
            return

        fix_sec  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._last_recv_stamp = fix_sec
        cov_e    = msg.position_covariance[0]
        cov_n    = msg.position_covariance[4]
        cov_ok   = msg.position_covariance_type > 0

        east  = (math.radians(msg.longitude - self._lon0)
                 * math.cos(math.radians(self._lat0)) * _R_EARTH)
        north = math.radians(msg.latitude - self._lat0) * _R_EARTH
        gps_enu = np.array([east, north], dtype=np.float64)

        rtk_ok = (cov_ok
                  and cov_e <= self._rtk_cov_max
                  and cov_n <= self._rtk_cov_max)

        vel_stamp = (self._last_vel.header.stamp.sec
                     + self._last_vel.header.stamp.nanosec * 1e-9
                     if self._last_vel is not None else -1e9)
        speed = 0.0
        if abs(fix_sec - vel_stamp) <= self._vel_max_age and self._last_vel is not None:
            lv = self._last_vel.twist.twist.linear
            speed = math.hypot(lv.x, lv.y)
        moving = speed >= self._speed_thr

        if rtk_ok and moving:
            self._last_rtk_wall = fix_sec
            if not self._gps_ok:
                self._on_gps_acquired()

        lio_pose: Optional[Tuple[float, float, float]] = None
        if rtk_ok and moving:
            lio_pose = self._interpolate_lio(fix_sec)

        # -- ALIGNING phase ---------------------------------------------------
        if self._state == _State.ALIGNING:
            if rtk_ok and moving and lio_pose is not None:
                x_l, y_l, psi_l = lio_pose
                lio_ant = self._lio_antenna(x_l, y_l, psi_l)
                weight  = 1.0 / (max(cov_e, 1e-9) + max(cov_n, 1e-9))
                self._calib_buf.append({
                    "lio_ant": lio_ant,
                    "gps_enu": gps_enu.copy(),
                    "weight":  weight,
                    "cov_e":   cov_e,
                    "cov_n":   cov_n,
                })
                if self._calib_last_gps is not None:
                    self._calib_path += float(
                        np.linalg.norm(gps_enu - self._calib_last_gps))
                self._calib_last_gps = gps_enu.copy()
                if self._calib_path >= self._calib_min:
                    self._do_alignment(self._calib_buf)
            return

        # -- ALIGNED phase ----------------------------------------------------
        if rtk_ok and moving and lio_pose is not None and self._gps_ok:
            x_l, y_l, psi_l = lio_pose
            lio_ant = self._lio_antenna(x_l, y_l, psi_l)
            weight  = 1.0 / (max(cov_e, 1e-9) + max(cov_n, 1e-9))

            if self._win_last_gps is not None:
                self._drift_since_solve += float(
                    np.linalg.norm(gps_enu - self._win_last_gps))

            self._win_append(lio_ant, gps_enu, weight, cov_e, cov_n)

            if (self._drift_since_solve >= self._drift_update_d
                    and len(self._win) >= self._min_win_pairs):
                self._update_transform()
                self._drift_since_solve = 0.0

    # =========================================================================
    # GPS quality state transitions
    # =========================================================================

    def _on_gps_acquired(self) -> None:
        self._gps_ok = True
        if self._state == _State.ALIGNED:
            self.get_logger().info(
                "lio_to_enu: GPS (re-)acquired -- clearing window")
            self._win_clear()

    def _gps_loss_cb(self) -> None:
        if not self._gps_ok or self._last_rtk_wall is None:
            return
        # Use header-stamp difference (rate-independent: unaffected by executor lag at
        # high bag-replay rates where the sim clock can run ahead of processed callbacks).
        elapsed = self._last_recv_stamp - self._last_rtk_wall
        if elapsed < self._gps_lost_to:
            return
        self._gps_ok = False
        if self._state == _State.ALIGNED:
            self.get_logger().info(
                f"lio_to_enu: GPS lost ({elapsed:.1f} s) -- transform frozen, window cleared")
            self._win_clear()
        elif self._state == _State.ALIGNING:
            # Keep pairs collected so far; only reset the position reference so
            # the GPS discontinuity at reacquisition doesn't inflate the path.
            self._calib_last_gps = None
            self.get_logger().info(
                f"lio_to_enu: GPS lost during alignment ({elapsed:.1f} s) -- "
                f"pausing ({len(self._calib_buf)} pairs, "
                f"{self._calib_path:.1f}/{self._calib_min:.1f} m collected)")

    # =========================================================================
    # Kabsch
    # =========================================================================

    def _lio_antenna(self, x: float, y: float, psi: float) -> np.ndarray:
        lever = _rot2d(psi) @ np.array([self._lx, self._ly])
        return np.array([x + lever[0], y + lever[1]], dtype=np.float64)

    def _do_alignment(self, buf: list) -> None:
        lio_ant = np.array([e["lio_ant"] for e in buf], dtype=np.float64)
        gps_enu = np.array([e["gps_enu"] for e in buf], dtype=np.float64)
        weights = np.array([e["weight"]  for e in buf], dtype=np.float64)
        path_len = self._calib_path

        R, t, theta = _kabsch_2d(lio_ant, gps_enu, weights)
        rms         = _kabsch_rms(lio_ant, gps_enu, R, t, weights)

        self._R     = R
        self._t     = t
        self._theta = theta

        for e in buf:
            self._win_append(e["lio_ant"], e["gps_enu"], e["weight"],
                             e["cov_e"],  e["cov_n"])

        self._calib_buf      = []
        self._calib_path     = 0.0
        self._calib_last_gps = None
        self._drift_since_solve = 0.0
        self._gps_ok = True
        self._state  = _State.ALIGNED

        cov_pos, cov_theta = self._combined_cov(rms)
        self._publish_transform(cov_pos, cov_theta)

        self.get_logger().info(
            f"lio_to_enu: ALIGNED  "
            f"theta={math.degrees(theta):+.2f} deg  "
            f"t=({t[0]:+.3f}, {t[1]:+.3f}) m  "
            f"n={len(buf)}  path={path_len:.1f} m  "
            f"RMS={rms * 100:.1f} cm  "
            f"cov_pos={cov_pos:.4f} m2")

    def _update_transform(self) -> None:
        if len(self._win) < self._min_win_pairs:
            return

        win_list = list(self._win)
        lio_ant  = np.array([e[0] for e in win_list], dtype=np.float64)
        gps_enu  = np.array([e[1] for e in win_list], dtype=np.float64)
        weights  = np.array([e[2] for e in win_list], dtype=np.float64)

        try:
            R_new, t_new, theta_new = _kabsch_2d(lio_ant, gps_enu, weights)
        except np.linalg.LinAlgError:
            self.get_logger().warn("lio_to_enu: SVD failed -- skipping update")
            return

        old_t     = self._t.copy() if self._t is not None else t_new.copy()
        old_theta = self._theta

        self._R     = R_new
        self._t     = t_new
        self._theta = theta_new

        rms = _kabsch_rms(lio_ant, gps_enu, R_new, t_new, weights)
        cov_pos, cov_theta = self._combined_cov(rms)
        self._publish_transform(cov_pos, cov_theta)

        delta_t   = float(np.linalg.norm(t_new - old_t))
        delta_deg = abs(math.degrees(_wrap(theta_new - old_theta)))
        self.get_logger().info(
            f"lio_to_enu: transform update  "
            f"dt={delta_t * 100:.1f} cm  dtheta={delta_deg:.3f} deg  "
            f"n={len(win_list)}  win_path={self._win_path:.1f} m  "
            f"RMS={rms * 100:.1f} cm  cov_pos={cov_pos:.4f} m2")

    def _combined_cov(self, rms: float) -> Tuple[float, float]:
        """Combined position and angular covariance for the Kabsch transform.

        cov_pos   = mean GPS xy cov over window + Kabsch MSE
        cov_theta = Kabsch MSE / (half window path)^2
        """
        if not self._win:
            return rms ** 2, rms ** 2
        win_list = list(self._win)
        mean_gps_cov = float(np.mean([(e[3] + e[4]) * 0.5 for e in win_list]))
        cov_pos   = mean_gps_cov + rms ** 2
        baseline  = max(self._win_path * 0.5, 1.0)
        cov_theta = rms ** 2 / (baseline ** 2)
        return cov_pos, cov_theta

    def _publish_transform(self, cov_pos: float, cov_theta: float) -> None:
        """Publish T: odom->map as nav_msgs/Odometry (transient_local)."""
        if self._R is None or self._t is None:
            return
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id  = "odom"
        msg.pose.pose.position.x = float(self._t[0])
        msg.pose.pose.position.y = float(self._t[1])
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _theta_to_quat(self._theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0]  = cov_pos   # xx
        msg.pose.covariance[7]  = cov_pos   # yy
        msg.pose.covariance[35] = cov_theta  # yaw-yaw
        for i in (14, 21, 28):
            msg.pose.covariance[i] = 1e6    # z, roll, pitch not estimated
        self._pub_transform.publish(msg)

    # =========================================================================
    # Sliding window
    # =========================================================================

    def _win_append(self, lio_ant: np.ndarray, gps_enu: np.ndarray,
                    weight: float, cov_e: float, cov_n: float) -> None:
        if self._win_last_gps is not None:
            self._win_path += float(np.linalg.norm(gps_enu - self._win_last_gps))
        self._win_last_gps = gps_enu.copy()
        self._win.append((lio_ant.copy(), gps_enu.copy(), weight, cov_e, cov_n))
        while self._win_path > self._drift_win_dist and len(self._win) > 2:
            oldest = self._win[0][1]
            nxt    = self._win[1][1]
            self._win_path -= float(np.linalg.norm(nxt - oldest))
            self._win.popleft()

    def _win_clear(self) -> None:
        self._win.clear()
        self._win_path         = 0.0
        self._win_last_gps     = None
        self._drift_since_solve = 0.0

    # =========================================================================
    # LIO interpolation
    # =========================================================================

    def _interpolate_lio(self, t: float) -> Optional[Tuple[float, float, float]]:
        buf = list(self._lio_buf)
        if len(buf) < 2:
            return None
        t_first, t_last = buf[0][0], buf[-1][0]
        if t < t_first:
            return None
        if t > t_last + 0.1:
            return None
        if t >= t_last:
            return (buf[-1][1], buf[-1][2], buf[-1][3])
        for i in range(len(buf) - 1, 0, -1):
            t0, t1 = buf[i - 1][0], buf[i][0]
            if t0 <= t <= t1:
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                _, x0, y0, p0 = buf[i - 1]
                _, x1, y1, p1 = buf[i]
                return (
                    x0 + alpha * (x1 - x0),
                    y0 + alpha * (y1 - y0),
                    _wrap(p0 + alpha * _wrap(p1 - p0)),
                )
        return None

    # =========================================================================
    # Watchdog
    # =========================================================================

    def _watchdog_cb(self) -> None:
        if self._state in (_State.ALIGNED, _State.WAITING_DATUM):
            return
        if self._datum_wall_sec is None:
            return
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._datum_wall_sec
        if elapsed > self._calib_timeout:
            self.get_logger().warn(
                f"lio_to_enu: initial alignment not achieved after {elapsed:.0f} s.  "
                f"Collected {len(self._calib_buf)} pairs, "
                f"path={self._calib_path:.2f}/{self._calib_min:.1f} m.  "
                f"Ensure the robot moves with RTK-fixed GPS.",
                throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = LioToEnu()
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
