#!/usr/bin/env python3
"""
kabsch_align.py — Post-hoc 2D alignment of LIO and GPS trajectories.

Reads /odometry/lio and /odometry/gps from a _processed bag (output of
offline_odometry.launch.py).  Both topics are already in the odom frame, so
the Kabsch result shows the residual error after gps_to_enu initial alignment.

Selects a clean segment (skip startup transient, require low GPS covariance,
require steady motion), fits the optimal 2D rigid transform R, t mapping
LIO → GPS via weighted Kabsch, and reports residuals.

Optional: writes a new bag with /odometry/lio rewritten as the aligned
trajectory so PlotJuggler can overlay them.

When to run
-----------
  Run on a _processed bag to evaluate how well gps_to_enu's initial bearing
  is aligned.  A large θ residual means the GPS bearing was locked at an
  incorrect heading — consider adjusting gps_to_enu's bearing_samples or
  speed_threshold.  A large translational residual that grows over time
  suggests lever-arm drift — re-check the lever_arm_x/y values in
  config/sensor_mount.yaml.

Usage
-----
  # Inspect the alignment on an existing processed bag:
  python3 kabsch_align.py /path/to/bag_processed

  # Same, plus write an aligned output bag:
  python3 kabsch_align.py /path/to/bag_processed --write-aligned

  # Override topics (e.g., on a _fastlio bag using raw FAST-LIO output):
  python3 kabsch_align.py /path/to/bag_fastlio \
      --lio-topic /Odometry --gps-topic /odometry/gps

Requirements
------------
  rosbag2_py, rclpy, numpy, scipy (for KDTree-based time matching).
  Run inside a sourced ROS 2 environment.
"""

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

import rclpy.serialization
import rosbag2_py
from nav_msgs.msg import Odometry


# ── Defaults — tune via CLI flags ───────────────────────────────────────────
DEFAULT_SKIP_DISTANCE   = 0.0   # m — discard first N metres of motion (startup transient)
DEFAULT_MAX_GPS_SIGMA   = 0.50   # m — reject GPS samples with σ > this (RTK: ~0.02 m, GNSS float: ~0.10–0.50 m)
DEFAULT_MIN_SPEED       = 0.4    # m/s — require steady forward motion
DEFAULT_SEGMENT_LENGTH  = 50.0   # m — length of fitting segment (after skip)
DEFAULT_TIME_TOL        = 0.05   # s — max time gap when matching LIO ↔ GPS samples


# ─────────────────────────────────────────────────────────────────────────────
# Bag I/O
# ─────────────────────────────────────────────────────────────────────────────

def read_odometry_topic(bag_path: str, topic: str):
    """Yield (t_sec, x, y, sigma_xx, sigma_yy) tuples from an Odometry topic."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise RuntimeError(f'Topic {topic} not found in {bag_path}. '
                           f'Available: {list(type_map.keys())}')

    msg_type = type_map[topic]
    if msg_type != 'nav_msgs/msg/Odometry':
        raise RuntimeError(f'Topic {topic} is {msg_type}, expected nav_msgs/msg/Odometry')

    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    out = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        msg = rclpy.serialization.deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Diagonal of pose covariance (xx, yy) — index 0 and 7 in row-major 6×6.
        sxx = msg.pose.covariance[0]
        syy = msg.pose.covariance[7]
        out.append((t, x, y, sxx, syy))
    arr = np.array(out, dtype=np.float64)
    return arr.reshape(-1, 5)  # ensure shape is (N, 5) even when empty


# ─────────────────────────────────────────────────────────────────────────────
# Time alignment
# ─────────────────────────────────────────────────────────────────────────────

def match_by_time(lio: np.ndarray, gps: np.ndarray, tol: float):
    """
    For each GPS sample, find the closest LIO sample in time within `tol`.
    Returns paired arrays (lio_xy, gps_xy, gps_sigma, t).

    GPS is the reference because its rate (~10 Hz) is lower than LIO (~10 Hz
    too, but LIO is what we're correcting).  Linear interpolation of LIO at
    GPS timestamps would be marginally cleaner; nearest-neighbour is good
    enough at these rates.
    """
    t_lio = lio[:, 0]
    t_gps = gps[:, 0]

    # For each GPS time, binary-search for the nearest LIO sample
    idx = np.searchsorted(t_lio, t_gps)
    idx = np.clip(idx, 1, len(t_lio) - 1)
    left  = t_lio[idx - 1]
    right = t_lio[idx]
    use_left = (t_gps - left) < (right - t_gps)
    nearest = np.where(use_left, idx - 1, idx)
    dt = np.abs(t_lio[nearest] - t_gps)
    keep = dt < tol

    paired_lio = lio[nearest][keep][:, 1:3]
    paired_gps = gps[keep][:, 1:3]
    paired_sigma = gps[keep][:, 3:5]   # σ_xx, σ_yy
    paired_t = t_gps[keep]

    return paired_lio, paired_gps, paired_sigma, paired_t


# ─────────────────────────────────────────────────────────────────────────────
# Segment selection
# ─────────────────────────────────────────────────────────────────────────────

def select_segment(lio_xy, gps_xy, gps_sigma, t,
                   skip_distance, max_sigma, segment_length):
    """
    Walk the trajectory, accumulate distance from start, and return indices
    that satisfy:
      - cumulative distance > skip_distance (past startup transient),
      - cumulative distance < skip_distance + segment_length,
      - per-axis GPS sigma < max_sigma (RTK-fix only).

    Distance is measured along the GPS trajectory, since GPS is the reference
    we're aligning LIO to.
    """
    diffs = np.diff(gps_xy, axis=0)
    seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
    cum_dist = np.concatenate(([0.0], np.cumsum(seg_lens)))

    in_segment = (cum_dist > skip_distance) & \
                 (cum_dist < skip_distance + segment_length)
    fix_quality = (np.sqrt(gps_sigma[:, 0]) < max_sigma) & \
                  (np.sqrt(gps_sigma[:, 1]) < max_sigma)
    keep = in_segment & fix_quality

    return keep, cum_dist


# ─────────────────────────────────────────────────────────────────────────────
# Kabsch (2D, weighted)
# ─────────────────────────────────────────────────────────────────────────────

def kabsch_2d(P, Q, weights=None):
    """
    Find R (2×2 rotation), t (2-vector) minimising
        Σ w_i ||R·p_i + t − q_i||²

    Args:
        P: (N, 2) source points (LIO).
        Q: (N, 2) target points (GPS).
        weights: (N,) non-negative weights.  If None, uniform.

    Returns:
        R: (2, 2) rotation matrix.
        t: (2,) translation vector.
        theta: float — rotation angle (radians, CCW).
    """
    if weights is None:
        weights = np.ones(len(P))
    w = weights / weights.sum()

    # Weighted centroids
    p_bar = (w[:, None] * P).sum(axis=0)
    q_bar = (w[:, None] * Q).sum(axis=0)

    # Centred clouds
    P_c = P - p_bar
    Q_c = Q - q_bar

    # Cross-covariance — note transpose so H is 2×2
    H = (P_c * w[:, None]).T @ Q_c

    # SVD
    U, _, Vt = np.linalg.svd(H)

    # Sign correction to avoid reflection
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, d])
    R = Vt.T @ D @ U.T

    t = q_bar - R @ p_bar
    theta = math.atan2(R[1, 0], R[0, 0])
    return R, t, theta


# ─────────────────────────────────────────────────────────────────────────────
# Reporting
# ─────────────────────────────────────────────────────────────────────────────

def report(R, t, theta, P, Q, weights, mask, full_lio, full_gps):
    """Print fit quality and residual statistics."""
    print()
    print('━' * 72)
    print(' Kabsch alignment result')
    print('━' * 72)
    print(f'  Rotation θ      : {math.degrees(theta):+.4f}°')
    print(f'  Translation t   : ({t[0]:+.4f}, {t[1]:+.4f}) m')
    print(f'  Fit samples     : {mask.sum()} / {len(mask)} '
          f'({100.0 * mask.sum() / len(mask):.1f}%)')

    # Residuals on fit segment
    P_aligned = (R @ P.T).T + t
    res = P_aligned - Q
    res_norm = np.linalg.norm(res, axis=1)
    w_norm = weights / weights.sum()
    rms = math.sqrt((w_norm * res_norm**2).sum())
    print(f'  Fit RMS residual: {rms*100:.2f} cm')
    print(f'  Fit max residual: {res_norm.max()*100:.2f} cm')

    # Residuals on full trajectory (unweighted, for diagnostic)
    full_aligned = (R @ full_lio.T).T + t
    full_res = np.linalg.norm(full_aligned - full_gps, axis=1)
    print()
    print('  Full-trajectory residual after alignment (unweighted):')
    print(f'    median: {np.median(full_res)*100:.2f} cm')
    print(f'    p90   : {np.percentile(full_res, 90)*100:.2f} cm')
    print(f'    p99   : {np.percentile(full_res, 99)*100:.2f} cm')
    print(f'    max   : {full_res.max()*100:.2f} cm')
    print('━' * 72)
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Optional: write aligned bag
# ─────────────────────────────────────────────────────────────────────────────

def write_aligned_bag(input_bag, output_bag, R, t, lio_topics=None):
    """
    Copy input bag verbatim, except rewrite every topic in `lio_topics` so
    their poses are aligned to GPS.  Both /odometry/lio (used by ekf_local)
    and /odometry/lio/global (used by ekf_global as an absolute anchor) must
    receive the same correction so both EKF stages start from a consistent
    frame.

    Twist is left untouched — it is in the body frame (child_frame_id) so it
    is unaffected by a rotation of the world/odom frame.

    Note: header.frame_id stays as-is; only the numbers are corrected.
    """
    if lio_topics is None:
        lio_topics = ['/odometry/lio']
    lio_topics_set = set(lio_topics)
    if Path(output_bag).exists():
        raise RuntimeError(f'Output bag already exists: {output_bag}')

    storage_options_in = rosbag2_py.StorageOptions(uri=input_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options_in, converter_options)

    storage_options_out = rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3')
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options_out, converter_options)

    # Re-create all topics in the output bag
    for topic_meta in reader.get_all_topics_and_types():
        writer.create_topic(topic_meta)

    R3 = np.eye(3)
    R3[:2, :2] = R
    t3 = np.array([t[0], t[1], 0.0])

    n_rewritten: dict = {tp: 0 for tp in lio_topics_set}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic in lio_topics_set:
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            p = np.array([msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z])
            p_aligned = R3 @ p + t3
            msg.pose.pose.position.x = float(p_aligned[0])
            msg.pose.pose.position.y = float(p_aligned[1])
            msg.pose.pose.position.z = float(p_aligned[2])

            # Rotate the orientation: q_new = q_R · q_old, where q_R is the
            # quaternion form of R3.  For a pure 2D yaw rotation θ:
            #   q_R = (0, 0, sin(θ/2), cos(θ/2))
            theta = math.atan2(R[1, 0], R[0, 0])
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            q  = msg.pose.pose.orientation
            # Hamilton product: q_new = q_R * q_old (rotation about z)
            new_x =  qw * q.x - qz * q.y
            new_y =  qw * q.y + qz * q.x
            new_z =  qw * q.z + qz * q.w
            new_w =  qw * q.w - qz * q.z
            msg.pose.pose.orientation.x = new_x
            msg.pose.pose.orientation.y = new_y
            msg.pose.pose.orientation.z = new_z
            msg.pose.pose.orientation.w = new_w

            # Twist is in body frame (child_frame_id), so rotation R between
            # odom frames does NOT affect twist values.  Leave untouched.

            data = rclpy.serialization.serialize_message(msg)
            n_rewritten[topic] += 1

        writer.write(topic, data, ts)

    for tp, count in sorted(n_rewritten.items()):
        print(f'  Rewrote {count} messages on {tp}')
    print(f'  Aligned bag written to {output_bag}')


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', help='Path to _processed bag (containing /odometry/lio and /odometry/gps).')
    parser.add_argument('--lio-topic',       default='/odometry/lio',
                        help='LIO odometry topic (default: /odometry/lio).')
    parser.add_argument('--gps-topic',       default='/odometry/gps',
                        help='GPS odometry topic (default: /odometry/gps).')
    parser.add_argument('--skip-distance',   type=float, default=DEFAULT_SKIP_DISTANCE,
                        help=f'Skip first N m of motion (default: {DEFAULT_SKIP_DISTANCE} m).')
    parser.add_argument('--segment-length',  type=float, default=DEFAULT_SEGMENT_LENGTH,
                        help=f'Use this many metres for fitting (default: {DEFAULT_SEGMENT_LENGTH} m).')
    parser.add_argument('--max-gps-sigma',   type=float, default=DEFAULT_MAX_GPS_SIGMA,
                        help=f'Reject GPS samples with σ_xy > this (default: {DEFAULT_MAX_GPS_SIGMA} m).')
    parser.add_argument('--time-tol',        type=float, default=DEFAULT_TIME_TOL,
                        help=f'Max time gap when matching samples (default: {DEFAULT_TIME_TOL} s).')
    parser.add_argument('--write-aligned',   action='store_true',
                        help='Write an aligned bag at <bag>_aligned with /Odometry rewritten.')
    args = parser.parse_args()

    bag = os.path.expanduser(args.bag.rstrip('/'))
    if not Path(bag).exists():
        sys.exit(f'Bag not found: {bag}')

    print(f'Reading {args.lio_topic} and {args.gps_topic} from {bag}...')
    lio = read_odometry_topic(bag, args.lio_topic)
    gps = read_odometry_topic(bag, args.gps_topic)
    print(f'  {len(lio)} LIO samples, {len(gps)} GPS samples')

    if len(gps) == 0:
        sys.exit(f'No GPS odometry samples found on topic {args.gps_topic} — '
                 f'cannot perform alignment (GPS may not have initialised).')
    if len(lio) == 0:
        sys.exit(f'No LIO odometry samples found on topic {args.lio_topic}.')

    # Time-match GPS to nearest LIO
    lio_xy, gps_xy, gps_sigma, t = match_by_time(lio, gps, args.time_tol)
    print(f'  {len(lio_xy)} time-matched pairs (tol={args.time_tol*1000:.0f} ms)')

    if len(lio_xy) < 50:
        sys.exit('Too few matched samples to fit reliably (<50).')

    # Select fit segment
    mask, cum_dist = select_segment(
        lio_xy, gps_xy, gps_sigma, t,
        args.skip_distance, args.max_gps_sigma, args.segment_length,
    )
    n_segment = mask.sum()
    print(f'  Segment selection: skip={args.skip_distance} m, '
          f'length={args.segment_length} m, max σ={args.max_gps_sigma*100:.1f} cm')
    if n_segment > 0:
        print(f'  → {n_segment} samples selected '
              f'(distance range: {cum_dist[mask].min():.1f} – {cum_dist[mask].max():.1f} m)')
    else:
        print(f'  → 0 samples selected (no segment matched the criteria)')

    if n_segment < 50:
        sys.exit('Too few samples in fit segment — try larger --segment-length '
                 'or relax --max-gps-sigma.')

    # Inverse-variance weights from GPS covariance (clip floor to avoid 1/0)
    sigma2 = np.maximum(gps_sigma[mask, 0] + gps_sigma[mask, 1], 1e-6)
    weights = 1.0 / sigma2

    P = lio_xy[mask]
    Q = gps_xy[mask]

    R, t_vec, theta = kabsch_2d(P, Q, weights)

    report(R, t_vec, theta, P, Q, weights, mask, lio_xy, gps_xy)

    if args.write_aligned:
        out_bag = bag + '_aligned'
        print(f'Writing aligned bag → {out_bag}')
        # Rewrite both LIO topics: /odometry/lio feeds ekf_local (differential)
        # and /odometry/lio/global feeds ekf_global (absolute anchor).  Both
        # are in the same odom/LIO frame and need the same correction.
        lio_topics_to_align = [args.lio_topic, '/odometry/lio/global']
        # Deduplicate in case --lio-topic was already /odometry/lio/global
        lio_topics_to_align = list(dict.fromkeys(lio_topics_to_align))
        write_aligned_bag(bag, out_bag, R, t_vec, lio_topics=lio_topics_to_align)


if __name__ == '__main__':
    rclpy.init(args=[])
    try:
        main()
    finally:
        rclpy.shutdown()