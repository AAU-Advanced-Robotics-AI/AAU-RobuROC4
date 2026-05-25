"""
offline_odometry.launch.py — Stage 2: frame relay + GPS conversion + Kabsch alignment (fast, iterate)

Three-stage offline pipeline
-----------------------------
  Stage 1  (offline_fastlio.launch.py, bottleneck — run ONCE per recording):
      raw bag → FAST-LIO2 → <bag>_fastlio

  Stage 2  (this file, fast ~20x — iterate on relay covariance, Kabsch, GPS params):
      <bag>_fastlio → lio_relay + gps_to_enu + lio_to_enu + gps_relay
      → <bag>_odometry

  Stage 3  (offline_ekf.launch.py, fast ~5x — iterate on EKF params):
      <bag>_odometry → ekf_local + ekf_global → <bag>_ekf

What is tunable at this stage
------------------------------
  lio_relay covariance  (config/localization.yaml, lio_relay section):
      cov_pos, cov_rot, cov_vel, cov_omega — how much the EKF trusts LIO
      relative to GPS.  Inflate if GPS corrections are too slow; deflate if
      the EKF drifts too far from GPS.

  lio_to_enu Kabsch params  (config/localization.yaml or inline):
      calib_min_baseline — metres of RTK-quality motion needed before the
        first alignment is published.  Increase if early GPS noise causes a
        poor initial alignment.
      lever_arm_x / y — loaded from config/sensor_mount.yaml.

  datum_lat / datum_lon  (launch args, default nan):
      Pin the ENU origin to a fixed GPS coordinate for cross-session route
      repeatability.  Leave as nan to auto-set from the first RTK-quality fix.

Data flow
---------
  <bag>_fastlio (/Odometry)
      └──→ lio_relay ─────────────────────────── /odometry/lio
                                                  (odom → base_link, REP-105)
                                              ── /odometry/lio/global
                                                  (map → base_link, post-Kabsch)

  <bag>_fastlio (/ublox_gps_node/fix, /fix_velocity)
      └──→ gps_to_enu ──────────────────────── /odometry/gps/raw
                                                  (map → gps, raw antenna ENU)
                                               ── /localization/datum  (transient_local)
            └──→ lio_to_enu ──────────────── /localization/lio_to_enu  (transient_local)
            └──→ gps_relay ──────────────── /odometry/gps
                                                  (map → base_link, lever-arm corrected)
                                               ── /odometry/gps/local
                                                  (body-frame velocity, local EKF)

Static TFs (from <bag>_fastlio bag replay)
------------------------------------------
  Stage 1 records all /tf_static messages into the bag, including the two
  published by fast_lio.launch.py (odom→camera_init, livox_frame→body) and
  the URDF-sourced transform (base_link→livox_frame).  When the bag is
  replayed here, all three arrive automatically — no explicit TF publishers
  are needed.

Recorded topics in <bag>_odometry
------------------------------------
  /odometry/lio              — relay output (odom → base_link, differential source)
  /odometry/lio/global       — relay output (map → base_link, post-Kabsch)
  /odometry/gps/raw          — raw antenna ENU position (map → gps)
  /odometry/gps              — lever-arm corrected position (map → base_link)
  /odometry/gps/local        — body-frame GPS velocity (local EKF input)
  /localization/datum        — ENU origin NavSatFix (transient_local)
  /localization/lio_to_enu   — Kabsch transform odom→map (transient_local)
  /ublox_gps_node/fix        — raw NavSatFix (covariance reference)
  /ublox_gps_node/fix_velocity — GPS ENU velocity
  /tf_static                 — passed through from <bag>_fastlio
  /robot_description         — passed through from <bag>_fastlio

Usage
-----
  ros2 launch roburoc_localization offline_odometry.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_fastlio

  # Higher speed (check for "late message" warnings in the terminal)
  ros2 launch roburoc_localization offline_odometry.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_fastlio rate:=30.0

  # Pin ENU datum for cross-session route repeatability
  ros2 launch roburoc_localization offline_odometry.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_fastlio \\
      datum_lat:=57.01234 datum_lon:=9.98765

  # Dry-run — stream into PlotJuggler without writing a bag
  ros2 launch roburoc_localization offline_odometry.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_fastlio record:=false

Notes
-----
  - Source only the AAU-RobuROC4 workspace (lio_ws not needed here).
  - If <bag>_odometry already exists, ros2 bag record will fail — delete it first.
  - Lever-arm and GPS antenna offset loaded from config/sensor_mount.yaml;
    update that file if the GPS antenna is remounted.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _load_mount():
    """Read sensor_mount.yaml and return the full dict (parse time, before launch)."""
    path = os.path.join(
        get_package_share_directory('roburoc_localization'), 'config', 'sensor_mount.yaml'
    )
    with open(path) as f:
        return yaml.safe_load(f)


# Loaded once at parse time so all values are available as Python constants.
_MOUNT = _load_mount()
_GPS   = _MOUNT['gps_antenna']


def _launch_setup(context, *args, **kwargs):
    bag       = os.path.expanduser(context.launch_configurations['bag'].rstrip('/'))
    rate      = context.launch_configurations['rate']
    record    = context.launch_configurations['record'].lower() in ('true', '1', 'yes')
    datum_lat = context.launch_configurations.get('datum_lat', 'nan')
    datum_lon = context.launch_configurations.get('datum_lon', 'nan')

    out_bag = bag.removesuffix('_fastlio') + '_odometry'

    loc_config = os.path.join(
        get_package_share_directory('roburoc_localization'), 'config', 'localization.yaml'
    )

    nodes = [

        # ── LIO relay: /Odometry → /odometry/lio[/global] ────────────────────
        # Frame-translates FAST-LIO's camera_init→body output to REP-105
        # odom→base_link and map→base_link, injecting parameterised covariance.
        # Parameters loaded from localization.yaml [lio_relay] section.
        Node(
            package='roburoc_localization',
            executable='lio_relay.py',
            name='lio_relay',
            output='screen',
            parameters=[loc_config, {'use_sim_time': True}],
        ),

        # ── GPS → ENU cartesian ───────────────────────────────────────────────
        # Converts NavSatFix → ENU Odometry, sets datum from first RTK fix.
        # Publishes /odometry/gps/raw (raw antenna position) and
        # /localization/datum (transient_local, consumed by lio_to_enu).
        Node(
            package='roburoc_localization',
            executable='gps_to_enu.py',
            name='gps_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'datum_lat':    float(datum_lat),
                'datum_lon':    float(datum_lon),
            }],
        ),

        # ── LIO/GPS Kabsch alignment ──────────────────────────────────────────
        # Online SE(2) Kabsch alignment of LIO odom frame → GPS ENU frame.
        # State machine: WAITING_DATUM → ALIGNING → ALIGNED (after 8 m baseline).
        # Publishes /localization/lio_to_enu (transient_local, consumed by
        # lio_relay and gps_relay) and /odometry/gps (lever-arm corrected
        # base_link position in map frame, global EKF input).
        Node(
            package='roburoc_localization',
            executable='lio_to_enu.py',
            name='lio_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'lever_arm_x':  float(_GPS['lever_arm_x']),
                'lever_arm_y':  float(_GPS['lever_arm_y']),
                'datum_lat':    float(datum_lat),
                'datum_lon':    float(datum_lon),
            }],
        ),

        # ── GPS relay: position + velocity → base_link frame ────────────────
        # Applies lever-arm correction to /odometry/gps/raw and publishes
        # /odometry/gps (base_link position in map frame, global EKF input).
        # Also rotates fix_velocity (ENU) → body frame and publishes
        # /odometry/gps/local (velocity-only, local EKF input).
        Node(
            package='roburoc_localization',
            executable='gps_relay.py',
            name='gps_relay',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'lever_arm_x':  float(_GPS['lever_arm_x']),
                'lever_arm_y':  float(_GPS['lever_arm_y']),
            }],
        ),

        # ── Bag replay (delayed 2 s to let nodes initialise) ──────────────────
        LogInfo(msg='Waiting 2 s for relay nodes to initialise before starting bag replay...'),
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg=f'Starting bag replay at rate={rate}: {bag}'),
                (bag_play := ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play', bag,
                        '--clock',
                        '--rate', rate,
                    ],
                    output='screen',
                )),
            ],
        ),

        # ── Auto-shutdown when bag replay finishes ────────────────────────────
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play,
                on_exit=[LogInfo(msg='Bag replay finished — shutting down.'), Shutdown()],
            )
        ),
    ]

    # ── Record processed odometry ────────────────────────────────────────────
    if record:
        nodes[0:0] = [
            LogInfo(msg=f'Recording processed odometry to: {out_bag}'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', out_bag,
                    '/odometry/lio',
                    '/odometry/lio/global',
                    '/odometry/gps/raw',
                    '/odometry/gps',
                    '/odometry/gps/local',
                    '/localization/datum',
                    '/localization/lio_to_enu',
                    '/ublox_gps_node/fix',
                    '/ublox_gps_node/fix_velocity',
                    '/tf_static',
                    '/robot_description',
                    '--use-sim-time',
                ],
                output='screen',
            ),
        ]

    return nodes


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'bag',
            description=(
                'Path to a _fastlio rosbag directory '
                '(output of offline_fastlio.launch.py).'
            ),
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='20.0',
            description=(
                'Bag playback rate.  lio_relay, gps_to_enu, and '
                'gps_relay are all pure maths; 20x is conservative.'
            ),
        ),
        DeclareLaunchArgument(
            'datum_lat',
            default_value='nan',
            description=(
                'Fixed datum latitude (degrees) for cross-session route repeatability.  '
                'Leave as nan to auto-set from the first RTK-quality fix.'
            ),
        ),
        DeclareLaunchArgument(
            'datum_lon',
            default_value='nan',
            description=(
                'Fixed datum longitude (degrees).  Must be provided together with datum_lat.'
            ),
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description=(
                'Write processed odometry to <bag>_odometry.  '
                'Set false to stream live into PlotJuggler without writing a bag.'
            ),
        ),

        OpaqueFunction(function=_launch_setup),

    ])
