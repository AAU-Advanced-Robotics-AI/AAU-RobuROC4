#!/usr/bin/env python3
"""
gps_covariance_inflator.py

Relay node that multiplies all covariance coefficients of a
nav_msgs/Odometry message by a scalar before republishing.

Placed between navsat_transform (/odometry/gps) and the local EKF,
this makes GPS act as a soft drift leash in the local frame rather
than a hard position constraint.  The global EKF receives the original,
unscaled message and stays tightly GPS-anchored.

Parameters:
    scale (float, default 5.0):
        Factor applied to every element of pose.covariance.
        Higher = more trust in LIO, less GPS pulling power.
        At RTK-fixed accuracy (~1 cm), scale=5 makes the EKF treat
        GPS as ~2 cm accuracy — still meaningful but not dominant.

Subscribed topics:
    ~/input  (nav_msgs/Odometry)  — unscaled GPS odometry

Published topics:
    ~/output (nav_msgs/Odometry)  — covariance-inflated GPS odometry
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GpsCovarianceInflator(Node):

    def __init__(self):
        super().__init__('gps_covariance_inflator')
        self.declare_parameter('scale', 2.0)
        self._scale = self.get_parameter('scale').value

        self._sub = self.create_subscription(
            Odometry, 'input', self._callback, 10)
        self._pub = self.create_publisher(Odometry, 'output', 10)

        self.get_logger().info(
            f'GPS covariance inflator ready — scale={self._scale}')

    def _callback(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist = msg.twist

        out.pose.covariance = [v * self._scale for v in msg.pose.covariance]

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsCovarianceInflator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
