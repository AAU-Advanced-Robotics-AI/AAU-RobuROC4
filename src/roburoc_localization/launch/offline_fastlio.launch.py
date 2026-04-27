"""
offline_fastlio.launch.py — Stage 1: FAST-LIO scan matching (slow, run once)

Three-stage offline pipeline
-----------------------------
  Stage 1  (this file, bottleneck — run ONCE per recording):
      raw bag → FAST-LIO2 → <bag>_fastlio

  Stage 2  (offline_odometry.launch.py, fast ~20x — iterate on relay/GPS params):
      <bag>_fastlio → lio_relay + gps_to_enu + gps_covariance_adapter
      → <bag>_processed

  Stage 3  (offline_ekf.launch.py, fast ~5x — iterate on EKF params):
      <bag>_processed → ekf_local + ekf_global → <bag>_ekf

Why the split?
--------------
FAST-LIO scan matching is the only CPU bottleneck; typical offline rate is
0.5-1x.  All downstream nodes (lio_relay, gps_to_enu, EKF) are pure maths
and run at 5-20x.  Keeping FAST-LIO in its own stage avoids re-running scan
matching every time a covariance value changes.

Recorded topics in <bag>_fastlio
----------------------------------
  /Odometry                    — raw FAST-LIO pose+twist (camera_init → body)
  /ublox_gps_node/fix          — raw NavSatFix
  /ublox_gps_node/fix_velocity — GPS ENU ground velocity
  /tf_static                   — all static TFs including odom→camera_init and
                                 livox_frame→body published by fast_lio.launch.py
  /robot_description           — URDF

Static TFs
----------
fast_lio.launch.py (included here) starts two static TF publishers
(odom→camera_init and livox_frame→body), whose values come from
config/sensor_mount.yaml.  Both are recorded into /tf_static in the output
bag so Stage 2 gets all required transforms from bag replay alone.

FAST-LIO with tf_en:false never looks up or writes the TF tree, so no TF
is needed for the fastlio_mapping node itself.

Usage
-----
  ros2 launch roburoc_localization offline_fastlio.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310

  # Dry-run (stream /Odometry live into PlotJuggler without writing a bag)
  ros2 launch roburoc_localization offline_fastlio.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310 record:=false

Notes
-----
  - Both workspaces must be sourced (lio_ws for fast_lio, AAU-RobuROC4 for
    roburoc_localization).
  - If <bag>_fastlio already exists, ros2 bag record will fail — delete it first.
  - Lower rate:=0.5 if FAST-LIO drops messages (watch for "drop message"
    warnings in the terminal).
  - Ctrl-C once bag playback finishes to flush and close the output bag.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _launch_setup(context, *args, **kwargs):
    bag    = os.path.expanduser(context.launch_configurations['bag'].rstrip('/'))
    rate   = context.launch_configurations['rate']
    record = context.launch_configurations['record'].lower() in ('true', '1', 'yes')

    out_bag = bag + '_fastlio'

    fast_lio_launch = os.path.join(
        get_package_share_directory('roburoc_localization'), 'launch', 'fast_lio.launch.py'
    )

    nodes = [

        # ── FAST-LIO + static TFs (via fast_lio.launch.py) ────────────────────
        # Starts fastlio_mapping (via upstream mapping.launch.py) and the two
        # static TF publishers odom→camera_init and livox_frame→body.
        # The static TFs are recorded into /tf_static in the output bag so that
        # Stage 2 receives all required transforms from bag replay alone.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # ── Bag replay (delayed 3 s to let FAST-LIO initialise) ───────────────
        LogInfo(msg='Waiting 3 s for FAST-LIO to initialise before starting bag replay...'),
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg=f'Starting bag replay at rate={rate}: {bag}'),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play', bag,
                        '--clock',
                        '--rate', rate,
                        '--topics',
                        '/livox/lidar',
                        '/livox/imu',
                        '/ublox_gps_node/fix',
                        '/ublox_gps_node/fix_velocity',
                        '/tf_static',
                        '/robot_description',
                    ],
                    output='screen',
                ),
            ],
        ),
    ]

    # ── Record FAST-LIO output + raw GPS ────────────────────────────────────
    if record:
        nodes[0:0] = [
            LogInfo(msg=f'Recording FAST-LIO output to: {out_bag}'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', out_bag,
                    '/Odometry',
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
            description='Absolute path to the raw input rosbag directory.',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description=(
                'Bag playback rate.  Lower to 0.5 if FAST-LIO drops messages '
                '(watch for "drop message" warnings in the terminal).'
            ),
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description=(
                'Write FAST-LIO output to <bag>_fastlio.  '
                'Set false to stream /Odometry live without writing a bag.'
            ),
        ),

        OpaqueFunction(function=_launch_setup),

    ])
