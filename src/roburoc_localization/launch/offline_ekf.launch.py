"""
offline_ekf.launch.py — Stage 3: fast EKF tuning against an odometry bag

Three-stage offline pipeline
-----------------------------
  Stage 1  (offline_fastlio.launch.py, bottleneck — run ONCE per recording):
      raw bag → FAST-LIO2 → <bag>_fastlio

  Stage 2  (offline_odometry.launch.py, fast ~20x — iterate on relay covariance,
            GPS params, gps_scale_*):
      <bag>_fastlio → lio_relay + gps_to_enu + gps_relay
      → <bag>_odometry

  Stage 3  (this file, fast ~5x — iterate on EKF params):
      <bag>_odometry → ekf_local + ekf_global → <bag>_ekf

Why three stages?
-----------------
Stage 1 is bottlenecked by FAST-LIO scan matching (~0.5-1x).  Stage 2 and 3
are pure maths and run at 20x and 5x respectively.  This split means a
10-minute drive can have its EKF tuned end-to-end in under 2 minutes.

Inputs from <bag>_odometry
---------------------------
  /odometry/lio          — LIO relay output (odom → base_link, differential)
  /odometry/lio/global   — LIO relay output (map → base_link, absolute)
  /odometry/gps          — GPS global output from gps_relay (tight anchor, x scale_global)
  /odometry/gps/local    — GPS local output from gps_relay (soft leash, x scale_local)
  /tf_static             — static TF tree (for robot frame lookups)
  /robot_description     — URDF

GPS covariance scaling
----------------------
  /odometry/gps/local is pre-computed by gps_relay in Stage 2.
  If you need to change gps_scale, re-run Stage 2 to regenerate the _odometry
  bag, then re-run Stage 3.  Changing EKF process/initial noise or
  sensor_timeout does NOT require re-running Stage 2.

EKF stack nodes
---------------
  ekf_local    fuses /odometry/lio (differential)
               → publishes /odom  (odom → base_link TF)
  ekf_global   fuses /odometry/lio/global + /odometry/gps
               → publishes /odometry/filtered/global  (map → odom TF)

Tuning workflow
---------------
  1. Edit config/localization.yaml (process_noise_covariance,
     initial_estimate_covariance, sensor_timeout).
  2. Re-run this launch against the same _odometry bag.
  3. Open PlotJuggler and compare /odom vs /odometry/gps/local & /odometry/lio.
  4. Repeat.

Usage
-----
  # Tune EKF with defaults (5x speed)
  ros2 launch roburoc_localization offline_ekf.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_odometry

  # Faster replay
  ros2 launch roburoc_localization offline_ekf.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_odometry rate:=8.0

  # Stream only — no output bag (useful with PlotJuggler live connection)
  ros2 launch roburoc_localization offline_ekf.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310_odometry record:=false
"""

import os

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


def _launch_setup(context, *args, **kwargs):
    bag    = os.path.expanduser(context.launch_configurations['bag'].rstrip('/'))
    rate   = context.launch_configurations['rate']
    record = context.launch_configurations['record'].lower() in ('true', '1', 'yes')

    out_bag = bag.removesuffix('_odometry') + '_ekf'

    loc_config = os.path.join(
        get_package_share_directory('roburoc_localization'), 'config', 'localization.yaml'
    )

    nodes = [

        # ── Local EKF: odom → base_link ──────────────────────────────────────
        # Fuses /odometry/lio (differential).
        # Publishes /odom and the odom → base_link TF.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[loc_config, {'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', '/odom'),
            ],
        ),

        # ── Global EKF: map → odom ────────────────────────────────────────────
        # Fuses /odometry/lio/global and /odometry/gps (global EKF — tight anchor).
        # /odometry/gps is already covariance-scaled by Stage 2 (scale_global=1.0)
        # Publishes /odometry/filtered/global and the map → odom TF.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[loc_config, {'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered/global'),
            ],
        ),

        # ── Bag replay (delayed 3 s to let EKF nodes initialise) ─────────────
        # Replay --all so /tf_static and /robot_description reach any subscriber.
        LogInfo(msg='Waiting 3 s for EKF nodes to initialise before starting bag replay...'),
        TimerAction(
            period=3.0,
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

    # ── Record EKF outputs ───────────────────────────────────────────────────
    if record:
        nodes[0:0] = [
            LogInfo(msg=f'Recording EKF outputs to: {out_bag}'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', out_bag,
                    '/odom',
                    '/odometry/filtered/global',
                    '/odometry/gps',           # pass-through (global EKF anchor)
                    '/odometry/gps/local',     # pass-through (local EKF soft leash)
                    '/odometry/lio',           # pass-through for comparison
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
                'Path to a _odometry rosbag directory '
                '(output of offline_odometry.launch.py).'
            ),
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='5.0',
            description=(
                'Bag playback rate.  The EKF has trivial CPU cost; '
                '5-10x is typically achievable on the NUC.'
            ),
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description=(
                'Record EKF outputs to <bag>_ekf.  '
                'Set false to stream live into PlotJuggler without writing a bag.'
            ),
        ),

        OpaqueFunction(function=_launch_setup),

    ])
