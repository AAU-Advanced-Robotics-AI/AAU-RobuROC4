"""
bag_process.launch.py — Offline bag processing: FAST_LIO + GPS-to-ENU conversion

Runs FAST_LIO and a lightweight GPS converter against a raw RobuROC4 bag
(sim clock), then records a compact processed bag with the derived odometry
topics ready for PlotJuggler comparison.

Data flow
---------
  bag (/livox/lidar, /livox/imu)
      └──→ FAST_LIO ──────────────────────────── /Odometry
                                                  (pose + twist, odom frame)
  bag (/ublox_gps_node/fix, /fix_velocity)
      └──→ gps_to_enu.py ────────────────────── /odometry/gps
                                                  (cartesian metres, odom frame)

GPS frame alignment (gps_to_enu.py, no /Odometry dependency)
------------------------------------------------------------
  Origin: first GPS fix → (0, 0), matching FAST_LIO's starting origin.
  Heading: once the robot exceeds speed_threshold m/s, GPS fix_velocity gives
  the compass bearing H (angle from North).  The ENU track is then rotated
  into FAST_LIO's odom frame:
      x_odom = east·sin(H) + north·cos(H)   (= robot-forward axis)
      y_odom = -east·cos(H) + north·sin(H)  (= robot-left axis)
  This derivation is purely from GPS — /Odometry is never needed.

Usage
-----
  # Process a bag — output written to <bag>_processed automatically
  ros2 launch roburoc_localization bag_process.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310

  # Dry-run (no recording — just stream live into PlotJuggler)
  ros2 launch roburoc_localization bag_process.launch.py \\
      bag:=$HOME/rosbags/roburoc_lio_20260423_201310 \\
      record:=false

Notes
-----
  - Both workspaces must be sourced (lio_ws for fast_lio, AAU-RobuROC4 for
    roburoc_localization and robot_localization).
  - If out_bag already exists, ros2 bag record will fail.  Delete or rename
    the existing directory first.
  - Increase rate:=1.0 if the machine keeps up; lower to 0.3 if FAST_LIO
    falls behind (watch for "drop message" warnings).
  - Press Ctrl-C once the bag playback finishes to flush and close the output bag.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bag    = os.path.expanduser(context.launch_configurations['bag'].rstrip('/'))
    rate   = context.launch_configurations['rate']
    record = context.launch_configurations['record'].lower() in ('true', '1', 'yes')

    # Derive output path: strip any trailing slash, append '_processed'
    out_bag = bag + '_processed'

    fast_lio_config = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'
    )

    nodes = [

        # ── FAST_LIO ─────────────────────────────────────────────────────────
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            parameters=[fast_lio_config, {'use_sim_time': True}],
        ),

        # ── GPS → local cartesian ────────────────────────────────────────────
        # gps_to_enu.py derives the ENU→odom frame rotation from GPS velocity at
        # first motion, with lever-arm correction to express positions at base_link.
        Node(
            package='roburoc_localization',
            executable='gps_to_enu.py',
            name='gps_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time':    True,
                'speed_threshold': 0.3,    # m/s — minimum GPS speed to start bearing estimation
                'bearing_samples': 5,      # number of velocity samples to average
                'bearing_timeout': 15.0,   # wall seconds before falling back to ENU
                # GPS antenna lever arm in base_link frame (matches livox_mid360_rtk.urdf.xacro).
                # Update these values if the antenna is remounted.
                'lever_arm_x':    -0.428,  # m (behind base_link centre)
                'lever_arm_y':     0.295,  # m (left of base_link centre)
            }],
        ),

        # ── Bag replay (delayed 3 s to let nodes initialise) ─────────────────
        # Only the topics that the processing nodes consume are replayed.
        LogInfo(msg=f'Waiting 3 s for FAST_LIO to initialise before starting bag replay...'),
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

    # ── Record processed topics ─────────────────────────────────────────────
    # Output bag contains everything needed for PlotJuggler analysis:
    #   /Odometry                    — FAST_LIO pose + twist (odom frame)
    #   /odometry/gps                — GPS in cartesian metres (odom frame, base_link)
    #   /ublox_gps_node/fix          — raw NavSatFix (covariance, fix type)
    #   /ublox_gps_node/fix_velocity — GPS ENU ground velocity
    if record:
        nodes[0:0] = [
            LogInfo(msg=f'Recording processed topics to: {out_bag}'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', out_bag,
                    '/Odometry',
                    '/odometry/gps',
                    '/ublox_gps_node/fix',
                    '/ublox_gps_node/fix_velocity',
                ],
                output='screen',
            ),
        ]

    return nodes


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'bag',
            description='Absolute path to the input rosbag directory.',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description=(
                'Bag playback rate.  0.5 is conservative and safe.  '
                'Increase to 1.0 if FAST_LIO keeps up without dropping messages.'
            ),
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Record processed topics to <bag>_processed.  Set false to stream live only.',
        ),

        OpaqueFunction(function=_launch_setup),

    ])
