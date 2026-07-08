#!/usr/bin/env python3
"""
export_bag_to_csv.py -- Dump localization topics from a ROS2 bag to CSVs.

Usage:
    pip install rosbags
    python3 export_bag_to_csv.py /path/to/bag_dir -o ./bag_csv
    # then zip the output folder and upload:
    cd ./bag_csv && zip -r ../bag_csv.zip .

Works on .db3 and .mcap bags. No ROS installation required (uses `rosbags`).

Each Odometry topic -> CSV with:
    t            bag receive time [s, relative to bag start]
    stamp        header.stamp [s, absolute]
    x, y, z      position
    yaw          heading from quaternion [rad]
    cov_xx, cov_yy, cov_xy, cov_yawyaw   pose covariance entries
    vx, vy, wz   twist linear x/y, angular z
    tcov_vxvx, tcov_vyvy, tcov_wzwz      twist covariance diagonals

NavSatFix -> t, stamp, status, lat, lon, alt, cov_ee, cov_nn, cov_type
TwistWithCovarianceStamped -> t, stamp, vx, vy, vz, cov_vxvx, cov_vyvy
"""

import argparse
import csv
import math
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

TOPICS = [
    "/odometry/filtered/global",
    "/odometry/lio/global",
    "/odometry/lio",
    "/odometry/gps",
    "/odometry/gps/raw",
    "/odometry/gps/local",
    "/localization/lio_to_enu",
    "/localization/datum",
    "/odom",
    "/ublox_gps_node/fix",
    "/ublox_gps_node/fix_velocity",
]

ODOM_HEADER = ["t", "stamp", "x", "y", "z", "yaw",
               "cov_xx", "cov_yy", "cov_xy", "cov_yawyaw",
               "vx", "vy", "wz",
               "tcov_vxvx", "tcov_vyvy", "tcov_wzwz"]
FIX_HEADER = ["t", "stamp", "status", "lat", "lon", "alt",
              "cov_ee", "cov_nn", "cov_type"]
TWIST_HEADER = ["t", "stamp", "vx", "vy", "vz", "cov_vxvx", "cov_vyvy"]


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def safe_name(topic: str) -> str:
    return topic.strip("/").replace("/", "__") + ".csv"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path, help="Path to the ROS2 bag directory")
    ap.add_argument("-o", "--out", type=Path, default=Path("./bag_csv"),
                    help="Output directory for CSVs (default ./bag_csv)")
    ap.add_argument("--topics", nargs="*", default=TOPICS,
                    help="Override the default topic list")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)

    with AnyReader([args.bag], default_typestore=TYPESTORE) as reader:
        available = {c.topic for c in reader.connections}
        wanted = [t for t in args.topics if t in available]
        missing = [t for t in args.topics if t not in available]
        if missing:
            print(f"NOTE: not in bag, skipping: {missing}", file=sys.stderr)
        if not wanted:
            print("ERROR: none of the requested topics are in this bag.",
                  file=sys.stderr)
            print(f"Bag contains: {sorted(available)}", file=sys.stderr)
            sys.exit(1)

        t0 = reader.start_time * 1e-9
        writers = {}
        files = {}
        counts = {t: 0 for t in wanted}

        try:
            conns = [c for c in reader.connections if c.topic in wanted]
            for conn, t_ns, raw in reader.messages(connections=conns):
                msg = reader.deserialize(raw, conn.msgtype)
                t_rel = t_ns * 1e-9 - t0
                topic = conn.topic

                if topic not in writers:
                    f = open(args.out / safe_name(topic), "w", newline="")
                    files[topic] = f
                    w = csv.writer(f)
                    if "Odometry" in conn.msgtype:
                        w.writerow(ODOM_HEADER)
                    elif "NavSatFix" in conn.msgtype:
                        w.writerow(FIX_HEADER)
                    else:
                        w.writerow(TWIST_HEADER)
                    writers[topic] = w
                w = writers[topic]

                if "Odometry" in conn.msgtype:
                    p = msg.pose.pose.position
                    c = msg.pose.covariance
                    tw = msg.twist.twist
                    tc = msg.twist.covariance
                    w.writerow([
                        f"{t_rel:.4f}", f"{stamp_to_sec(msg.header.stamp):.4f}",
                        p.x, p.y, p.z,
                        yaw_from_quat(msg.pose.pose.orientation),
                        c[0], c[7], c[1], c[35],
                        tw.linear.x, tw.linear.y, tw.angular.z,
                        tc[0], tc[7], tc[35],
                    ])
                elif "NavSatFix" in conn.msgtype:
                    c = msg.position_covariance
                    w.writerow([
                        f"{t_rel:.4f}", f"{stamp_to_sec(msg.header.stamp):.4f}",
                        msg.status.status, msg.latitude, msg.longitude,
                        msg.altitude, c[0], c[4],
                        msg.position_covariance_type,
                    ])
                else:  # TwistWithCovarianceStamped
                    lv = msg.twist.twist.linear
                    c = msg.twist.covariance
                    w.writerow([
                        f"{t_rel:.4f}", f"{stamp_to_sec(msg.header.stamp):.4f}",
                        lv.x, lv.y, lv.z, c[0], c[7],
                    ])
                counts[topic] += 1
        finally:
            for f in files.values():
                f.close()

    print(f"\nExported to {args.out}/")
    for t in wanted:
        print(f"  {t:45s} {counts[t]:7d} msgs")
    print("\nNow zip and upload:")
    print(f"  cd {args.out} && zip -r ../bag_csv.zip .")


if __name__ == "__main__":
    main()