"""
offline_odometry.launch.py — Stage 2: frame relay + GPS conversion + covariance scaling (fast, iterate)

Three-stage offline pipeline
-----------------------------
  Stage 1  (offline_fastlio.launch.py, bottleneck — run ONCE per recording):
      raw bag → FAST-LIO2 → <bag>_fastlio

  Stage 2  (this file, fast ~20x — iterate on relay covariance, GPS params, gps_scale_*):
      <bag>_fastlio → lio_relay + gps_to_enu + gps_relay
      → <bag>_odometry

  Stage 3  (offline_ekf.launch.py, fast ~5x — iterate on EKF params):
      <bag>_odometry → ekf_local + ekf_global → <bag>_ekf

What is tunable at this stage
------------------------------
  lio_relay covariance  (config/localization.yaml, lio_relay section):
      cov_pos, cov_rot, cov_vel, cov_omega — how much the EKF trusts LIO
      relative to GPS.  Inflate if GPS corrections are too slow; deflate if
      the EKF drifts too far from GPS.

  gps_to_enu bearing / lever-arm params  (launch args below):
      speed_threshold, bearing_samples — tighten or loosen the bearing-lock
      condition if the initial GPS track diverges from the LIO track.
      lever_arm_x / y — loaded from config/sensor_mount.yaml.

  gps_scale_global / gps_scale_local  (launch args, defaults 1.0 / 3.0):
      GPS covariance multipliers for the global and local EKF inputs.
      scale_global=1.0 trusts the receiver's covariance exactly (tight anchor).
      scale_local=3.0 makes GPS a soft drift leash so LIO dominates locally.
      GPS covariance scaling lives here (not in Stage 3) because it is a
      property of the GPS representation, not of the EKF tuning.  After
      changing gps_scale_*, re-run Stage 2 to regenerate the _odometry bag;
      Stage 3 can then re-run without touching the EKF parameters.

Data flow
---------
  <bag>_fastlio (/Odometry)
      └──→ lio_relay ─────────────────────────── /odometry/lio
                                                  (odom → base_link, REP-105)
                                              ── /odometry/lio/global
                                                  (map → base_link, absolute)

  <bag>_fastlio (/ublox_gps_node/fix, /fix_velocity)
      └──→ gps_to_enu ────────────────────────── /odometry/gps/raw
                                                  (odom → base_link, metres)
            └──→ gps_relay ──────────────────── /odometry/gps
                                                  (global EKF anchor, x scale_global)
                                              ── /odometry/gps/local
                                                  (local EKF soft leash, x scale_local)

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
  /odometry/lio/global       — relay output (map → base_link, absolute source)
  /odometry/gps              — GPS global output (tight anchor, x scale_global)
  /odometry/gps/local        — GPS local output (soft leash, x scale_local)
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

  # Tune GPS covariance scale
  ros2 launch roburoc_localization offline_odometry.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_fastlio gps_scale:=10.0

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
    scale_local  = context.launch_configurations['gps_scale_local']
    scale_global = context.launch_configurations['gps_scale_global']

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

        # ── GPS → local cartesian ─────────────────────────────────────────────
        # Derives ENU→odom rotation from GPS velocity compass bearing at first
        # motion; lever-arm correction uses yaw from /odometry/lio (odom frame).
        # Lever-arm values loaded from config/sensor_mount.yaml [gps_antenna].
        Node(
            package='roburoc_localization',
            executable='gps_to_enu.py',
            name='gps_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time':    True,
                'speed_threshold': 0.3,
                'bearing_samples': 5,
                'bearing_timeout': 15.0,
                'lever_arm_x':    float(_GPS['lever_arm_x']),
                'lever_arm_y':    float(_GPS['lever_arm_y']),
                'odom_topic':     '/odometry/lio',
            }],
        ),

        # ── GPS relay: /odometry/gps/raw → /odometry/gps + /odometry/gps/local ──────────
        # Publishes two covariance-scaled GPS streams from gps_to_enu's raw output:
        #   /odometry/gps        (global EKF — tight anchor, scale_global)
        #   /odometry/gps/local  (local EKF — soft drift leash, scale_local)
        # Mirrors lio_relay: primary topic = most common use case, /sub = secondary.
        # Tuning gps_scale_* is a Stage 2 operation: after changing, re-run Stage 2
        # to regenerate the _odometry bag; Stage 3 iterates on EKF params alone.
        Node(
            package='roburoc_localization',
            executable='gps_relay.py',
            name='gps_relay',
            output='screen',
            parameters=[loc_config, {
                'use_sim_time': True,
                'scale_local':  float(scale_local),
                'scale_global': float(scale_global),
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
            'gps_scale_global',
            default_value='1.0',
            description=(
                'GPS covariance scale for the global EKF (/odometry/gps).  '
                '1.0 trusts the receiver covariance exactly.  Raise to loosen the '
                'GPS anchor on the map frame.  Changing requires re-running Stage 2.'
            ),
        ),
        DeclareLaunchArgument(
            'gps_scale_local',
            default_value='3.0',
            description=(
                'GPS covariance scale for the local EKF (/odometry/gps/local).  '
                'High value (3) makes GPS a soft drift leash; LIO dominates '
                'scan-to-scan motion.  Changing requires re-running Stage 2.'
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
