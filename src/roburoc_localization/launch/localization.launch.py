"""
Localization launch — FAST-LIO + Dual EKF + NavSat Transform

Fuses FAST-LIO2 LiDAR-inertial odometry with u-blox ZED-F9P RTK-GPS to
produce both local and global state estimates:

  lio_relay              /Odometry (FAST-LIO2) → /odometry/lio + /odometry/lio/global
  navsat_transform       GPS lat/lon → local cartesian /odometry/gps/raw
  gps_relay              /odometry/gps/raw → /odometry/gps (global) + /odometry/gps/local (soft leash)
  ekf_local              odom → base_link   (smooth, for control)
                         remapped to /odom for downstream consumers
  ekf_global             map  → odom        (GPS-anchored, for teach-and-repeat)
                         published on /odometry/filtered/global

Data flow:
  /Odometry (FAST-LIO2) ──→ lio_relay ──→ /odometry/lio       → ekf_local  (used as differential)
                                       └──→ /odometry/lio/global → ekf_global (used as absolute)
  /ublox_gps_node/fix → navsat_transform → /odometry/gps/raw → gps_relay → /odometry/gps     → ekf_global (tight anchor)
                                                                           └→ /odometry/gps/local → ekf_local (soft leash)

Prerequisites:
  - robot.launch.py running (TF tree from URDF, sensor drivers)

NOTE: GPS EKF fusion (odom1 entries) is currently commented out in
  localization.yaml — each EKF runs on LIO relay output only.
  Re-enable and tune gps_scale before using GPS as an active fusion source.

Usage:
  ros2 launch roburoc_localization localization.launch.py
  ros2 launch roburoc_localization localization.launch.py gps_scale:=8.0
  ros2 launch roburoc_localization localization.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('roburoc_localization')
    config_file = os.path.join(pkg_localization, 'config', 'localization.yaml')
    fast_lio_launch = os.path.join(pkg_localization, 'launch', 'fast_lio.launch.py')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    gps_scale_local  = LaunchConfiguration('gps_scale_local')
    gps_scale_global = LaunchConfiguration('gps_scale_global')
    rviz_use       = LaunchConfiguration('rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (for bag replay with --clock)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz alongside FAST-LIO',
        ),
        DeclareLaunchArgument(
            'gps_scale_global',
            default_value='1.0',
            description=(
                'GPS covariance scale for the global EKF (/odometry/gps).  '
                '1.0 trusts the receiver covariance exactly.'
            ),
        ),
        DeclareLaunchArgument(
            'gps_scale_local',
            default_value='3.0',
            description=(
                'GPS covariance scale for the local EKF (/odometry/gps/local).  '
                'High value makes GPS a soft drift leash; LIO dominates '
                'scan-to-scan motion.'
            ),
        ),

        # ── FAST-LIO2 + static TFs ───────────────────────────────────────
        # Includes fast_lio.launch.py which in turn includes the upstream
        # mapping.launch.py and adds the two static TF publishers:
        #   odom → camera_init  and  livox_frame → body
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz':         rviz_use,
            }.items(),
        ),

        # ── LIO relay: FAST-LIO2 → conformant odom streams ───────────────
        # Subscribes: /Odometry  (camera_init → body, FAST-LIO2 native)
        # Publishes:  /odometry/lio         (odom → base_link, for local EKF)
        #             /odometry/lio/global  (map  → base_link, for global EKF)
        # Requires: odom→camera_init and livox_frame→body static TFs from
        #           fast_lio.launch.py, and base_link→livox_frame from URDF.
        Node(
            package='roburoc_localization',
            executable='lio_relay.py',
            name='lio_relay',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),

        # ── NavSat Transform: GPS → local cartesian ─────────────────────
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('gps/fix',           '/ublox_gps_node/fix'),
                ('odometry/filtered', '/odometry/filtered/global'),
                ('odometry/gps',      '/odometry/gps/raw'),
                ('gps/filtered',      '/gps/filtered'),
            ],
        ),

        # ── GPS relay (local + global covariance scaling) ───────────────────────
        # Takes /odometry/gps/raw from navsat_transform and publishes two
        # covariance-adjusted streams (mirrors lio_relay's dual output):
        #   /odometry/gps        — global EKF input (tight anchor, ×scale_global)
        #   /odometry/gps/local  — local EKF input (soft leash, ×scale_local)
        Node(
            package='roburoc_localization',
            executable='gps_relay.py',
            name='gps_relay',
            output='screen',
            parameters=[config_file, {
                'scale_local':  gps_scale_local,
                'use_sim_time': use_sim_time,
                'scale_global': gps_scale_global,
            }],
        ),

        # ── Local EKF: odom → base_link ──────────────────────────────────
        # Output remapped to /odom — the reserved name for the local EKF
        # output consumed by controllers, Nav2, and visualization tools.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odom'),
            ],
        ),

        # ── Global EKF: map → odom ───────────────────────────────────────
        # Output on /odometry/filtered/global — used for path recording and
        # as the odometry input to navsat_transform (circular by design).
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered/global'),
            ],
        ),

    ])
