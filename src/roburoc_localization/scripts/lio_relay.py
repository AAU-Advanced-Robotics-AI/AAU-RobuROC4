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

Covariance strategy (current: constant)
----------------------------------------
FAST-LIO2's internal covariance is the propagated linearization uncertainty
of its tightly-coupled filter.  It is systematically overconfident: it does
not account for map-model error, scan degeneracy, or IMU bias drift, and can
be 10–100× smaller than the true error in open or featureless environments.

Current approach: constant diagonal covariance set via ROS parameters.
  - The local EKF uses LIO in differential mode (odom0_differential: true), so
    only the *step* uncertainty matters, not absolute position — the constant is
    a reasonable proxy for per-step noise.
  - The global EKF uses absolute mode, so the constant also determines how much
    GPS has to fight LIO to anchor the map frame.  Tune upward if GPS is slow
    to pull the estimate.

Planned upgrade (implement when constant proves insufficient):
  - Subscribe to /cloud_registered_body, compute eigen-decomposition of the 3D
    point covariance matrix.  When the minimum eigenvalue is near zero (flat
    ground plane, open field), inflate the horizontal-translation and yaw
    covariance entries.  This is a lightweight degeneracy proxy (~30 lines)
    that does not require FAST-LIO2 internals.

Parameters
----------
  cov_pos   (float, default 0.1):  Diagonal variance for x, y, z position [m²]
  cov_rot   (float, default 0.05): Diagonal variance for roll, pitch, yaw [rad²]
  cov_vel   (float, default 0.05): Diagonal variance for linear velocity [m²/s²]
  cov_omega (float, default 0.01): Diagonal variance for angular velocity [rad²/s²]

Prerequisites
-------------
  - robot_state_publisher publishing the URDF TF tree (base_link → livox_frame)
  - fast_lio.launch.py running (static odom→camera_init and livox_frame→body TFs)
  - FAST-LIO2 publishing /Odometry with publish.tf_en: false
"""

import numpy as np

import rclpy
from rclpy.node import Node
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

        # ── Covariance parameters ─────────────────────────────────────────────
        self.declare_parameter('cov_pos',   0.1)
        self.declare_parameter('cov_rot',   0.05)
        self.declare_parameter('cov_vel',   0.05)
        self.declare_parameter('cov_omega', 0.01)

        cp  = self.get_parameter('cov_pos').value
        cr  = self.get_parameter('cov_rot').value
        cv  = self.get_parameter('cov_vel').value
        co  = self.get_parameter('cov_omega').value

        self._pose_cov  = _build_diag_cov36([cp, cp, cp, cr, cr, cr])
        self._twist_cov = _build_diag_cov36([cv, cv, cv, co, co, co])

        self.get_logger().info(
            f'lio_relay: cov_pos={cp}, cov_rot={cr}, '
            f'cov_vel={cv}, cov_omega={co}')

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Static transforms cached after first successful lookup.
        self._T_odom_camerainit: tuple | None = None  # (translation, quaternion)
        self._T_body_baselink:   tuple | None = None

        # ── I/O ───────────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Odometry, '/Odometry', self._callback, 10)

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
                       wx: float, wy: float, wz: float) -> Odometry:
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
        out.pose.covariance = list(self._pose_cov)

        out.twist.twist.linear.x  = float(vx)
        out.twist.twist.linear.y  = float(vy)
        out.twist.twist.linear.z  = float(vz)
        out.twist.twist.angular.x = float(wx)
        out.twist.twist.angular.y = float(wy)
        out.twist.twist.angular.z = float(wz)
        out.twist.covariance = list(self._twist_cov)

        return out

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

        # Local EKF stream: frame_id=odom (used with odom0_differential: true)
        self._pub_local.publish(
            self._make_odometry(stamp, 'odom', t_ob, q_ob,
                                lv[0], lv[1], lv[2],
                                av[0], av[1], av[2]))

        # Global EKF stream: frame_id=map (absolute pose; at startup map==odom)
        self._pub_global.publish(
            self._make_odometry(stamp, 'map', t_ob, q_ob,
                                lv[0], lv[1], lv[2],
                                av[0], av[1], av[2]))


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
