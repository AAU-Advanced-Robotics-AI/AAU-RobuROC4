#!/usr/bin/env python3
"""
gps_relay.py

Relay node parallel to lio_relay: takes the raw GPS cartesian odometry from
gps_to_enu (/odometry/gps/raw) and publishes two covariance-adjusted streams
for the dual-EKF stack.

  /odometry/gps         — for the global EKF: covariance x scale_global
                          Tightly anchors the map frame; trusts the GPS
                          receiver's reported covariance at near-face value.

  /odometry/gps/local   — for the local EKF: covariance x scale_local
                          Acts as a soft drift leash so LIO incremental motion
                          dominates; GPS gently corrects long-term drift.

This mirrors the lio_relay pattern:
  /odometry/lio         — local EKF (differential mode)
  /odometry/lio/global  — global EKF (absolute mode)

Parameters
----------
  scale_local  (float, default 3.0):
      Covariance multiplier for the local EKF topic.  At RTK-fixed accuracy
      (~1 cm), scale=3 makes the EKF treat GPS as ~3 cm accuracy — LIO
      dominates scan-to-scan but GPS prevents long-term drift accumulation.
      Increase to loosen the GPS leash; decrease toward 1.0 to tighten it.

  scale_global (float, default 1.0):
      Covariance multiplier for the global EKF topic.  1.0 = trust the GPS
      receiver's reported covariance exactly.  Lower below 1.0 if the
      reported covariance is known to be conservative; raise to loosen the
      GPS anchor on the map frame.

Subscribed topics
-----------------
  /odometry/gps/raw   (nav_msgs/Odometry) — raw GPS cartesian from gps_to_enu

Published topics
----------------
  /odometry/gps        (nav_msgs/Odometry) — global EKF input (tight anchor)
  /odometry/gps/local  (nav_msgs/Odometry) — local EKF input (soft leash)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GpsRelay(Node):

    def __init__(self):
        super().__init__('gps_relay')
        self.declare_parameter('scale_local',  3.0)
        self.declare_parameter('scale_global',  1.0)
        self._scale_local  = self.get_parameter('scale_local').value
        self._scale_global = self.get_parameter('scale_global').value

        self._sub = self.create_subscription(
            Odometry, '/odometry/gps/raw', self._callback, 10)
        self._pub_global = self.create_publisher(Odometry, '/odometry/gps',       10)
        self._pub_local  = self.create_publisher(Odometry, '/odometry/gps/local', 10)

        self.get_logger().info(
            f'gps_relay ready — '
            f'scale_local={self._scale_local}, scale_global={self._scale_global}')

    def _scale_msg(self, msg: Odometry, scale: float, frame_id: str | None = None) -> Odometry:
        out = Odometry()
        out.header           = msg.header
        if frame_id is not None:
            out.header.frame_id = frame_id
        out.child_frame_id   = msg.child_frame_id
        out.pose.pose        = msg.pose.pose
        out.twist.twist      = msg.twist.twist
        out.pose.covariance  = [v * scale for v in msg.pose.covariance]
        out.twist.covariance = [v * scale for v in msg.twist.covariance]
        return out

    def _callback(self, msg: Odometry) -> None:
        # Global EKF (world_frame=map): frame_id must be 'map' to avoid the
        # circular TF dependency where ekf_global would look up map→odom using
        # the transform it itself publishes, causing exponential divergence.
        # GPS coordinates from gps_to_enu are computed in a fixed inertial frame
        # (origin = robot start, x = initial heading) — that IS the map frame.
        self._pub_local.publish(self._scale_msg(msg, self._scale_local))
        self._pub_global.publish(self._scale_msg(msg, self._scale_global, frame_id='map'))


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


if __name__ == '__main__':
    main()
