#!/usr/bin/env python3
"""
pgo_to_bag.py -- turn lidar_gps_pgo optimizer artifacts into a PlotJuggler-
replayable rosbag in the pipeline ENU frame.

The PGO backend writes optimized_poses_tum.txt in ITS OWN map frame plus
georeference.txt (datum + ENU->map yaw/translation).  This converts the final
(loop-closed) trajectory into the pipeline's ENU frame and records it as
nav_msgs/Odometry on /odometry/pgo/global -- directly overlayable with the EKF
stack's /odometry/*/global topics.

To keep everything in one PlotJuggler session (like offline_ekf passes through
comparison topics), the matching <base>_ekf bag's comparison topics are copied
in when present.

Usage:
    source /opt/ros/humble/setup.bash
    python3 pgo_to_bag.py <artifacts_dir> <out_bag> [--ekf <ekf_bag>]

    # e.g.
    python3 pgo_to_bag.py roburoc_lio_20260528_161115_pgo_artifacts \
                          roburoc_lio_20260528_161115_pgo \
                          --ekf roburoc_lio_20260528_161115_ekf
"""
import argparse
import shutil
from pathlib import Path

import numpy as np

import rclpy.serialization as rs
import rosbag2_py
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from builtin_interfaces.msg import Time

# ---- WGS84 geodetic <-> ENU (matches lidar_gps_pgo/src/geo_utils.cpp) --------
_A = 6378137.0
_E2 = 6.69437999014e-3


def _geodetic_to_ecef(lat, lon, alt):
    lat, lon = np.radians(lat), np.radians(lon)
    sl, cl = np.sin(lat), np.cos(lat)
    n = _A / np.sqrt(1.0 - _E2 * sl * sl)
    return np.array([(n + alt) * cl * np.cos(lon),
                     (n + alt) * cl * np.sin(lon),
                     (n * (1 - _E2) + alt) * sl])


def _R_enu_ecef(lat, lon):
    lat, lon = np.radians(lat), np.radians(lon)
    sl, cl, sp, cp = np.sin(lon), np.cos(lon), np.sin(lat), np.cos(lat)
    return np.array([[-sl, cl, 0.0],
                     [-sp * cl, -sp * sl, cp],
                     [cp * cl, cp * sl, sp]])


class _Geo:
    def __init__(self, lat, lon, alt):
        self.ecef0 = _geodetic_to_ecef(lat, lon, alt)
        self.R = _R_enu_ecef(lat, lon)

    def to_enu(self, lat, lon, alt):
        return self.R @ (_geodetic_to_ecef(lat, lon, alt) - self.ecef0)

    def enu_to_geodetic(self, enu):
        e = self.ecef0 + self.R.T @ enu
        b = _A * np.sqrt(1 - _E2)
        ep2 = (_A * _A - b * b) / (b * b)
        p = np.hypot(e[0], e[1])
        th = np.arctan2(_A * e[2], b * p)
        lon = np.arctan2(e[1], e[0])
        lat = np.arctan2(e[2] + ep2 * b * np.sin(th) ** 3,
                         p - _E2 * _A * np.cos(th) ** 3)
        n = _A / np.sqrt(1 - _E2 * np.sin(lat) ** 2)
        return np.degrees(lat), np.degrees(lon), p / np.cos(lat) - n


def _parse_georef(path):
    d = {}
    for line in Path(path).read_text().splitlines():
        if ":" in line and not line.startswith("#"):
            k, v = line.split(":", 1)
            d[k.strip()] = v.strip()
    lat, lon, alt = map(float, d["datum_lat_lon_alt"].split())
    # ENU<->map alignment is only present if it converged (align_max_rms). If the
    # GPS/odom fit never met the threshold, these keys are absent.
    aligned = "enu_to_map_yaw_rad" in d and "enu_to_map_t_xy" in d
    return dict(datum=(lat, lon, alt),
                aligned=aligned,
                yaw=float(d["enu_to_map_yaw_rad"]) if aligned else 0.0,
                t=(np.array(list(map(float, d["enu_to_map_t_xy"].split())))
                   if aligned else np.zeros(2)),
                z_off=float(d.get("enu_to_map_z_offset", 0.0)))


def _quat_mul(a, b):  # (x,y,z,w)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz])


def _time_msg(t):
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def _reader(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    return r


def _datum_from_ekf(ekf_bag):
    try:
        r = _reader(ekf_bag)
        r.set_filter(rosbag2_py.StorageFilter(topics=["/localization/datum"]))
        while r.has_next():
            _, data, _ = r.read_next()
            m = rs.deserialize_message(data, NavSatFix)
            return (m.latitude, m.longitude, m.altitude)
    except Exception:
        pass
    return None


# Comparison topics copied from the _ekf bag so one PlotJuggler session shows
# PGO, EKF, GPS and raw LIO together.
_PASSTHROUGH = {
    "/odometry/filtered/global": "nav_msgs/msg/Odometry",
    "/odometry/gps":             "nav_msgs/msg/Odometry",
    "/odometry/lio/global":      "nav_msgs/msg/Odometry",
    "/localization/datum":       "sensor_msgs/msg/NavSatFix",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("artifacts")
    ap.add_argument("out_bag")
    ap.add_argument("--ekf", default=None,
                    help="matching _ekf bag whose comparison topics are copied in")
    a = ap.parse_args()

    art = Path(a.artifacts)
    gr = _parse_georef(art / "georeference.txt")
    tum = np.loadtxt(art / "optimized_poses_tum.txt")
    if tum.ndim == 1:
        tum = tum[None, :]

    # map -> PGO-ENU (own datum) -> geodetic -> pipeline-ENU
    pipe_datum = (_datum_from_ekf(a.ekf) if a.ekf else None) or gr["datum"]
    geo_pgo, geo_pipe = _Geo(*gr["datum"]), _Geo(*pipe_datum)
    c, s = np.cos(gr["yaw"]), np.sin(gr["yaw"])
    Rt = np.array([[c, s], [-s, c]])                       # R(yaw)^T
    # map orientation -> ENU: pre-rotate by -yaw about Z
    hy = -gr["yaw"] / 2.0
    q_map_enu = np.array([0.0, 0.0, np.sin(hy), np.cos(hy)])  # (x,y,z,w)

    if not gr["aligned"]:
        print(f"[pgo_to_bag] WARNING: {art.name} has NO ENU<->map alignment "
              f"(GPS/odom fit never met align_max_rms). Writing /odometry/pgo/global "
              f"in the PGO map frame (frame_id=map_pgo) — it will NOT overlay with "
              f"the EKF/GPS ENU topics. Investigate this bag's FAST-LIO / GPS quality.")

    out = Path(a.out_bag)
    if out.exists():
        shutil.rmtree(out)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name="/odometry/pgo/global", type="nav_msgs/msg/Odometry",
        serialization_format="cdr"))

    frame_id = "map" if gr["aligned"] else "map_pgo"
    n = 0
    for row in tum:
        t = row[0]
        if gr["aligned"]:
            map_xy = row[1:3]
            enu_xy = Rt @ (map_xy - gr["t"])
            enu_z = row[3] - gr["z_off"]
            lat, lon, alt = geo_pgo.enu_to_geodetic(
                np.array([enu_xy[0], enu_xy[1], enu_z]))
            x, y, z = geo_pipe.to_enu(lat, lon, alt)
            q_enu = _quat_mul(q_map_enu, row[4:8])  # (x,y,z,w)
        else:
            # no alignment: keep the loop-closed poses in the PGO map frame
            x, y, z = row[1], row[2], row[3]
            q_enu = row[4:8]

        od = Odometry()
        od.header.stamp = _time_msg(t)
        od.header.frame_id = frame_id
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.position.z = float(z)
        od.pose.pose.orientation.x = float(q_enu[0])
        od.pose.pose.orientation.y = float(q_enu[1])
        od.pose.pose.orientation.z = float(q_enu[2])
        od.pose.pose.orientation.w = float(q_enu[3])
        writer.write("/odometry/pgo/global", rs.serialize_message(od),
                     int(round(t * 1e9)))
        n += 1

    # ---- pass through comparison topics from the _ekf bag ----
    copied = {}
    if a.ekf and Path(a.ekf).exists():
        from rosidl_runtime_py.utilities import get_message
        present = {}
        r0 = _reader(a.ekf)
        for tm in r0.get_all_topics_and_types():
            if tm.name in _PASSTHROUGH:
                present[tm.name] = tm.type
                writer.create_topic(rosbag2_py.TopicMetadata(
                    name=tm.name, type=tm.type, serialization_format="cdr"))
        if present:
            r = _reader(a.ekf)
            r.set_filter(rosbag2_py.StorageFilter(topics=list(present)))
            while r.has_next():
                topic, data, ts = r.read_next()
                writer.write(topic, data, ts)
                copied[topic] = copied.get(topic, 0) + 1

    print(f"[pgo_to_bag] {out.name}: /odometry/pgo/global={n} poses"
          + (f"  + passthrough {copied}" if copied else "  (no _ekf passthrough)"))


if __name__ == "__main__":
    main()
