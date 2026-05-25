#!/usr/bin/env python3
"""
gps_to_enu.py — Convert NavSatFix to ENU Odometry (map frame) for the live stack.

The first RTK-quality fix (position_covariance[0] < rtk_cov_max) becomes the
ENU datum / map-frame origin.  Subsequent fixes are expressed as flat-Earth ENU
offsets from that origin and published as the raw GPS ANTENNA position in the
map frame (child_frame_id=gps).

No lever-arm correction is applied here — that requires a globally consistent
heading which is not available until the Kabsch alignment in lio_to_enu is
solved.  gps_relay subscribes to /odometry/gps/raw and /localization/lio_to_enu
and publishes the lever-arm-corrected base_link position on /odometry/gps once
aligned.

The datum is also published once on /localization/datum (transient_local QoS) so
that lio_to_enu can receive it even if it starts after this node.

Publications
------------
  /odometry/gps/raw     nav_msgs/Odometry  frame_id=map  child=gps
  /localization/datum   sensor_msgs/NavSatFix  (transient_local, published once)

Subscriptions
-------------
  /ublox_gps_node/fix    sensor_msgs/NavSatFix

Parameters
----------
  rtk_cov_max   float  0.01  Max east covariance [m^2] to accept a datum fix
  sentinel_cov  float  1e6   Position covariance sentinel [m^2]
  datum_lat     float  nan   Fixed datum latitude  [deg] (optional, cross-session)
  datum_lon     float  nan   Fixed datum longitude [deg] (optional, cross-session)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

_R_EARTH = 6_378_137.0


class GpsToEnu(Node):

    def __init__(self):
        super().__init__("gps_to_enu")

        self.declare_parameter("rtk_cov_max",  0.01)
        self.declare_parameter("sentinel_cov", 1e6)
        self.declare_parameter("datum_lat",    float("nan"))
        self.declare_parameter("datum_lon",    float("nan"))

        self._rtk_cov_max = self.get_parameter("rtk_cov_max").value
        self._sentinel    = self.get_parameter("sentinel_cov").value

        _datum_lat = self.get_parameter("datum_lat").value
        _datum_lon = self.get_parameter("datum_lon").value

        if not (math.isnan(_datum_lat) or math.isnan(_datum_lon)):
            self._lat0 = _datum_lat
            self._lon0 = _datum_lon
            self.get_logger().info(
                f"gps_to_enu: fixed datum -- lat={_datum_lat:.6f} lon={_datum_lon:.6f}")
        else:
            self._lat0 = None
            self._lon0 = None

        self._datum_published: bool = False

        latch_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(NavSatFix, "/ublox_gps_node/fix", self._fix_cb, 10)

        self._pub_gps   = self.create_publisher(Odometry,  "/odometry/gps/raw",  10)
        self._pub_datum = self.create_publisher(NavSatFix, "/localization/datum", latch_qos)

        if self._lat0 is not None:
            self._publish_datum_now(NavSatFix(), self._lat0, self._lon0)

        self.get_logger().info(
            f"gps_to_enu: rtk_cov_max={self._rtk_cov_max} m2  "
            f"Publishing raw antenna positions on /odometry/gps/raw (child=gps). "
            f"Lever-arm correction is applied by lio_to_enu after Kabsch alignment.")

    def _fix_cb(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            return

        cov_e  = msg.position_covariance[0]
        cov_n  = msg.position_covariance[4]
        cov_ok = msg.position_covariance_type > 0

        if self._lat0 is None:
            if not cov_ok or cov_e > self._rtk_cov_max:
                self.get_logger().warn(
                    f"gps_to_enu: waiting for RTK datum -- "
                    f"east_std={math.sqrt(max(cov_e, 0.0)):.3f} m "
                    f"(need < {math.sqrt(self._rtk_cov_max):.3f} m)",
                    throttle_duration_sec=10.0)
                return
            self._lat0 = msg.latitude
            self._lon0 = msg.longitude
            self.get_logger().info(
                f"gps_to_enu: RTK datum set -- lat={self._lat0:.6f} lon={self._lon0:.6f}")
            self._publish_datum_now(msg, self._lat0, self._lon0)
            return

        east  = (math.radians(msg.longitude - self._lon0)
                 * math.cos(math.radians(self._lat0)) * _R_EARTH)
        north = math.radians(msg.latitude - self._lat0) * _R_EARTH

        # No lever-arm correction here — heading is unknown before Kabsch
        # alignment.  lio_to_enu applies the lever arm once aligned and
        # publishes the corrected base_link position on /odometry/gps.
        odom = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id  = "gps"
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        if cov_ok:
            odom.pose.covariance[0]  = cov_e
            odom.pose.covariance[7]  = cov_n
            odom.pose.covariance[14] = 0.01
        else:
            for i in (0, 7, 14):
                odom.pose.covariance[i] = self._sentinel

        for i in (21, 28, 35):
            odom.pose.covariance[i] = self._sentinel

        self._pub_gps.publish(odom)

    def _publish_datum_now(self, fix_msg: NavSatFix,
                           lat: float, lon: float) -> None:
        if self._datum_published:
            return
        datum = NavSatFix()
        datum.header.stamp    = self.get_clock().now().to_msg()
        datum.header.frame_id = "map"
        datum.latitude        = lat
        datum.longitude       = lon
        datum.altitude        = getattr(fix_msg, "altitude", 0.0) or 0.0
        datum.position_covariance_type = fix_msg.position_covariance_type
        datum.position_covariance      = list(fix_msg.position_covariance)
        self._pub_datum.publish(datum)
        self._datum_published = True
        self.get_logger().info(
            f"gps_to_enu: datum published on /localization/datum "
            f"(lat={lat:.6f}, lon={lon:.6f})")


def main(args=None):
    rclpy.init(args=args)
    node = GpsToEnu()
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
