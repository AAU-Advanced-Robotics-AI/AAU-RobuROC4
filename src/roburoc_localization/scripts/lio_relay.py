#!/usr/bin/env python3
"""
lio_relay.py

Relay node that converts FAST-LIO2's non-conformant odometry output into
two REP-105-conformant streams for the dual-EKF localization stack, and
injects well-formed covariance matrices.

FAST-LIO2 publishes:
  /Odometry  header.frame_id  = camera_init   (IMU pose at startup, world-fixed)
             child_frame_id   = body          (IMU/LiDAR frame, hardcoded in source)

This relay publishes:
  /odometry/lio         header.frame_id=odom, child_frame_id=base_link
                        For the local EKF (odom0_differential: true)
  /odometry/lio/global  header.frame_id=map,  child_frame_id=base_link
                        For the global EKF (absolute pose, odom0_differential: false)

Transform used per message
--------------------------
  T_odom_baselink = T_odom_camerainit · T_camerainit_body · T_body_baselink

  T_odom_camerainit — static TF odom → camera_init, published by fast_lio.launch.py
                      Equals the URDF base_link → livox_frame joint values.
  T_camerainit_body — from the incoming /Odometry pose (FAST-LIO2 estimate).
  T_body_baselink   — static TF body → base_link, resolved through
                      body → livox_frame (identity) → base_link (URDF).

Both static transforms are looked up once via tf2_ros after node startup.

Covariance strategy
-------------------
FAST-LIO2's internal covariance is the propagated linearization uncertainty
of its tightly-coupled filter.  It is systematically overconfident: it does
not account for map-model error, scan degeneracy, or IMU bias drift, and can
be 10–100× smaller than the true error in open or featureless environments.

Two different strategies are used for the two output channels:

Local channel (/odometry/lio, differential mode):
  The EKF only sees pose *deltas*, not absolute position.  The covariance
  therefore represents per-step uncertainty — how accurately did FAST-LIO2
  estimate the motion between consecutive scans?  For a MID360 in structured
  terrain this is typically 1–3 mm per 10 cm step and ~0.01 rad per 10 cm.
  A small constant is the right model here: it is approximately stationary
  (each step has similar quality) and it must be small enough that the EKF
  actually uses the LIO increments rather than ignoring them in favour of Q.
  Parameters: cov_pos_local, cov_rot_local, cov_vel, cov_omega.

Global channel (/odometry/lio/global, absolute mode):
  The EKF fuses the full absolute pose in the map frame.  Covariance now
  represents cumulative scan-matching error since startup — how far could the
  LIO map have drifted from truth?  This grows with distance travelled because:
    - Each scan-match has a small relative error ε ∝ step_size
    - Errors accumulate (roughly) as a random walk: σ_pos ≈ k_drift · √distance
    - More conservatively, in open/featureless terrain: σ_pos ≈ k_drift · distance
  We use the linear (worst-case) model with a minimum floor so GPS is never
  completely frozen out by an overconfident LIO at startup:
    cov_pos_global = max(cov_pos_floor², (k_drift_pos · total_distance)²)
    cov_rot_global = max(cov_rot_floor², (k_drift_rot · total_distance)²)
  Typical values (MID360, agricultural field):
    k_drift_pos = 0.02  →  2 cm per metre driven  (conservative for open field)
    k_drift_rot = 0.0003 rad/m  →  ~1.7° per 100 m
  These give Kalman gains that let GPS override LIO once the robot has driven
  a few metres away from the start, while LIO dominates for the first few
  metres when GPS may not yet have RTK lock.
  Parameters: cov_pos_floor, cov_rot_floor, k_drift_pos, k_drift_rot.

Planned upgrade (degeneracy-aware scaling):
  Subscribe to /cloud_registered_body, compute eigen-decomposition of the 3D
  point covariance matrix.  The ratio λ_min/λ_max is a degeneracy proxy:
  near 0 → flat/featureless (inflate xy and yaw), near 1 → rich geometry.
  Can replace or multiply k_drift for a physics-aware per-scan weight.

Parameters
----------
  # Local channel (constant per-step noise):
  cov_pos_local  (float, default 0.001):  Diagonal position variance [m²]
  cov_rot_local  (float, default 0.0001): Diagonal rotation variance [rad²]
  cov_vel        (float, default 0.01):   Diagonal linear velocity variance [(m/s)²]
  cov_omega      (float, default 0.001):  Diagonal angular velocity variance [(rad/s)²]

  # Global channel (distance-proportional, with floor):
  cov_pos_floor  (float, default 0.01):   Minimum position variance [m²] (= floor of 0.1 m σ)
  cov_rot_floor  (float, default 0.0001): Minimum rotation variance [rad²] (= floor of 0.01 rad σ)
  k_drift_pos    (float, default 0.02):   Position drift rate [m error / m travelled]
  k_drift_rot    (float, default 0.0003): Rotation drift rate [rad / m travelled]

Prerequisites
-------------
  - robot_state_publisher publishing the URDF TF tree (base_link → livox_frame)
  - fast_lio.launch.py running (static odom→camera_init and livox_frame→body TFs)
  - FAST-LIO2 publishing /Odometry with publish.tf_en: false
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import tf2_ros

from nav_msgs.msg import Odometry


# ── Quaternion helpers ────────────────────────────────────────────────────────

def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions represented as [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def _quat_conj(q: np.ndarray) -> np.ndarray:
    """Conjugate of [x, y, z, w] — same as inverse for unit quaternions."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by unit quaternion q [x, y, z, w]."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    return _quat_mul(_quat_mul(q, qv), _quat_conj(q))[:3]


def _compose(t1: np.ndarray, q1: np.ndarray,
             t2: np.ndarray, q2: np.ndarray):
    """Compose SE(3) transforms (t1, q1) · (t2, q2) → (t, q)."""
    return t1 + _quat_rotate(q1, t2), _quat_mul(q1, q2)


def _tf_to_arrays(tf_transform):
    """Convert a geometry_msgs/Transform to (translation [3,], quat [x,y,z,w])."""
    tr = tf_transform.translation
    ro = tf_transform.rotation
    return np.array([tr.x, tr.y, tr.z]), np.array([ro.x, ro.y, ro.z, ro.w])


def _pose_to_arrays(pose):
    """Convert a geometry_msgs/Pose to (translation [3,], quat [x,y,z,w])."""
    p = pose.position
    o = pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])


def _build_diag_cov36(diag_values: list[float]) -> list[float]:
    """Build a flat 6×6 covariance from 6 diagonal values."""
    cov = [0.0] * 36
    for i, v in enumerate(diag_values):
        cov[i * 7] = v   # diagonal indices: 0, 7, 14, 21, 28, 35
    return cov


# ── Node ─────────────────────────────────────────────────────────────────────

class LioRelay(Node):

    def __init__(self):
        super().__init__('lio_relay')

        # ── Covariance parameters — local channel (constant per-step) ─────────
        self.declare_parameter('cov_pos_local',  0.001)
        self.declare_parameter('cov_rot_local',  0.0001)
        self.declare_parameter('cov_vel',        0.01)
        self.declare_parameter('cov_omega',      0.001)

        # ── Covariance parameters — global channel (distance-proportional) ────
        self.declare_parameter('cov_pos_floor',  0.01)    # 0.1 m σ floor
        self.declare_parameter('cov_rot_floor',  0.0001)  # 0.01 rad σ floor
        self.declare_parameter('k_drift_pos',    0.02)    # m error / m travelled
        self.declare_parameter('k_drift_rot',    0.0003)  # rad / m travelled

        cp_l = self.get_parameter('cov_pos_local').value
        cr_l = self.get_parameter('cov_rot_local').value
        cv   = self.get_parameter('cov_vel').value
        co   = self.get_parameter('cov_omega').value

        self._cov_pos_floor = self.get_parameter('cov_pos_floor').value
        self._cov_rot_floor = self.get_parameter('cov_rot_floor').value
        self._k_drift_pos   = self.get_parameter('k_drift_pos').value
        self._k_drift_rot   = self.get_parameter('k_drift_rot').value

        # Local channel: fixed covariance (per-step noise)
        self._local_pose_cov  = _build_diag_cov36([cp_l, cp_l, cp_l, cr_l, cr_l, cr_l])
        self._twist_cov       = _build_diag_cov36([cv,   cv,   cv,   co,   co,   co  ])

        self.get_logger().info(
            f'lio_relay local:  cov_pos={cp_l}, cov_rot={cr_l}, '
            f'cov_vel={cv}, cov_omega={co}')
        self.get_logger().info(
            f'lio_relay global: floor_pos={self._cov_pos_floor}, '
            f'floor_rot={self._cov_rot_floor}, '
            f'k_drift_pos={self._k_drift_pos}, k_drift_rot={self._k_drift_rot}')

        # ── Kabsch transform from /localization/lio_to_enu ────────────────────
        self._kabsch_odom: Odometry | None = None   # latest Kabsch Odometry msg
        self._cov_kabsch_pos: float = 0.0
        self._cov_kabsch_rot: float = 0.0
        # Distance since last Kabsch update (odom frame) for covariance growth
        self._d_since_kabsch: float = 0.0
        self._last_t_ob: np.ndarray | None = None  # previous base_link position (odom)

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Static transforms cached after first successful lookup.
        self._T_odom_camerainit: tuple | None = None  # (translation, quaternion)
        self._T_body_baselink:   tuple | None = None

        # ── I/O ───────────────────────────────────────────────────────────────
        latch_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._sub = self.create_subscription(
            Odometry, '/Odometry', self._callback, 10)
        self.create_subscription(
            Odometry, '/localization/lio_to_enu', self._kabsch_cb, latch_qos)

        self._pub_local  = self.create_publisher(Odometry, '/odometry/lio', 10)
        self._pub_global = self.create_publisher(Odometry, '/odometry/lio/global', 10)

        self.get_logger().info('lio_relay: waiting for static TF transforms...')

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _lookup_static_transforms(self) -> bool:
        """Try to cache both static transforms. Returns True when both are ready."""
        try:
            t_oc = self._tf_buffer.lookup_transform(
                'odom', 'camera_init', rclpy.time.Time())
            t_bb = self._tf_buffer.lookup_transform(
                'body', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return False

        self._T_odom_camerainit = _tf_to_arrays(t_oc.transform)
        self._T_body_baselink   = _tf_to_arrays(t_bb.transform)
        self.get_logger().info('lio_relay: static transforms acquired — relay active.')
        return True

    def _make_odometry(self, stamp, frame_id: str,
                       t_ob: np.ndarray, q_ob: np.ndarray,
                       vx: float, vy: float, vz: float,
                       wx: float, wy: float, wz: float,
                       pose_cov: list[float]) -> Odometry:
        out = Odometry()
        out.header.stamp    = stamp
        out.header.frame_id = frame_id
        out.child_frame_id  = 'base_link'

        out.pose.pose.position.x    = float(t_ob[0])
        out.pose.pose.position.y    = float(t_ob[1])
        out.pose.pose.position.z    = float(t_ob[2])
        out.pose.pose.orientation.x = float(q_ob[0])
        out.pose.pose.orientation.y = float(q_ob[1])
        out.pose.pose.orientation.z = float(q_ob[2])
        out.pose.pose.orientation.w = float(q_ob[3])
        out.pose.covariance = list(pose_cov)

        out.twist.twist.linear.x  = float(vx)
        out.twist.twist.linear.y  = float(vy)
        out.twist.twist.linear.z  = float(vz)
        out.twist.twist.angular.x = float(wx)
        out.twist.twist.angular.y = float(wy)
        out.twist.twist.angular.z = float(wz)
        out.twist.covariance = list(self._twist_cov)

        return out

    def _kabsch_cb(self, msg: Odometry) -> None:
        """Receive updated Kabsch transform from lio_to_enu."""
        self._kabsch_odom    = msg
        self._cov_kabsch_pos = msg.pose.covariance[0]
        self._cov_kabsch_rot = msg.pose.covariance[35]
        self._d_since_kabsch = 0.0   # reset drift accumulator on new estimate
        self.get_logger().info(
            f'lio_relay: Kabsch transform updated -- '
            f'cov_pos={self._cov_kabsch_pos:.4f} m2')

    def _kabsch_pose_cov(self) -> list[float]:
        """Grow pose covariance from the Kabsch baseline as the robot moves.

        cp = cov_kabsch_pos + (k_drift_pos * d_since_kabsch)^2
        cr = cov_kabsch_rot + (k_drift_rot * d_since_kabsch)^2
        """
        d  = self._d_since_kabsch
        cp = self._cov_kabsch_pos + (self._k_drift_pos * d) ** 2
        cr = self._cov_kabsch_rot + (self._k_drift_rot * d) ** 2
        return _build_diag_cov36([cp, cp, cp, cr, cr, cr])

    # ── Subscription callback ─────────────────────────────────────────────────

    def _callback(self, msg: Odometry) -> None:
        # FAST-LIO2 publishes: frame_id=camera_init, child_frame_id=body

        if self._T_odom_camerainit is None or self._T_body_baselink is None:
            if not self._lookup_static_transforms():
                self.get_logger().warn(
                    'lio_relay: static TF not yet available, dropping message',
                    throttle_duration_sec=5.0)
                return

        # T_camerainit_body from the incoming Odometry pose
        t_cb, q_cb = _pose_to_arrays(msg.pose.pose)

        # T_odom_baselink = T_odom_camerainit · T_camerainit_body · T_body_baselink
        t_oc, q_oc = self._T_odom_camerainit
        t_bb, q_bb = self._T_body_baselink

        t_tmp, q_tmp = _compose(t_oc, q_oc, t_cb, q_cb)
        t_ob,  q_ob  = _compose(t_tmp, q_tmp, t_bb, q_bb)

        # Accumulate odom-frame distance for Kabsch covariance growth.
        if self._last_t_ob is not None:
            step = float(np.linalg.norm(t_ob - self._last_t_ob))
            self._d_since_kabsch += step
        self._last_t_ob = t_ob.copy()

        # Rotate twist from body frame into base_link frame.
        # Lever-arm velocity correction is second-order for a near-CoG mounting
        # and is omitted; add if the IMU is far from base_link.
        lv = _quat_rotate(q_bb, [msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z])
        av = _quat_rotate(q_bb, [msg.twist.twist.angular.x,
                                  msg.twist.twist.angular.y,
                                  msg.twist.twist.angular.z])

        stamp = msg.header.stamp

        # Local EKF stream: constant per-step covariance (frame_id=odom, differential)
        self._pub_local.publish(
            self._make_odometry(stamp, 'odom', t_ob, q_ob,
                                lv[0], lv[1], lv[2],
                                av[0], av[1], av[2],
                                self._local_pose_cov))

        # Global EKF stream: Kabsch-corrected map-frame pose.
        # Only publish once a Kabsch transform is available.
        if self._kabsch_odom is not None:
            k = self._kabsch_odom
            t_k = np.array([k.pose.pose.position.x,
                            k.pose.pose.position.y,
                            k.pose.pose.position.z])
            q_k = np.array([k.pose.pose.orientation.x,
                            k.pose.pose.orientation.y,
                            k.pose.pose.orientation.z,
                            k.pose.pose.orientation.w])
            t_map, q_map = _compose(t_k, q_k, t_ob, q_ob)
            self._pub_global.publish(
                self._make_odometry(stamp, 'map', t_map, q_map,
                                    lv[0], lv[1], lv[2],
                                    av[0], av[1], av[2],
                                    self._kabsch_pose_cov()))


def main(args=None):
    rclpy.init(args=args)
    node = LioRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
