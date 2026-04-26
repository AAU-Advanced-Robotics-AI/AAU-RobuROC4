"""
Localization launch — Dual EKF + NavSat Transform

Fuses FAST-LIO2 LiDAR-inertial odometry with u-blox ZED-F9P RTK-GPS to
produce both local and global state estimates:

  ekf_local              odom → base_link   (smooth, for control)
  ekf_global             map  → odom        (GPS-anchored, for teach-and-repeat)
  navsat_transform       GPS lat/lon → local cartesian /odometry/gps
  gps_covariance_inflator /odometry/gps → /odometry/gps/inflated
                             (inflated covariance for the local EKF so GPS
                              acts as a soft drift leash, not a hard constraint)

Data flow:
  /ublox_gps_node/fix → navsat_transform → /odometry/gps ──┬──→ ekf_global (full covariance)
                                                            └──→ inflator → /odometry/gps/inflated
                                                                         → ekf_local  (soft leash)

Prerequisites:
  - robot.launch.py running (TF tree, sensor drivers)
  - FAST-LIO2 running with publish.tf_en: false (publishes /Odometry only)

NOTE: GPS EKF fusion (odom1 entries) is currently commented out in
  dual_ekf_navsat.yaml — each EKF runs on FAST-LIO2 /Odometry only.
  Re-enable and tune gps_scale before using GPS as an active fusion source.

Usage:
  ros2 launch roburoc_localization localization.launch.py
  ros2 launch roburoc_localization localization.launch.py gps_scale:=8.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('roburoc_localization')
    config_file = os.path.join(pkg_localization, 'config', 'dual_ekf_navsat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gps_scale    = LaunchConfiguration('gps_scale')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (for bag replay with --clock)',
        ),
        DeclareLaunchArgument(
            'gps_scale',
            default_value='2.0',
            description=(
                'Scalar multiplied onto all GPS covariance coefficients fed to the '
                'local EKF.  Higher = more trust in LIO, GPS is a softer drift leash. '
                'Tune in the field: increase if GPS causes jitter, decrease if local '
                'odometry drifts in open field.'
            ),
        ),

        # ── NavSat Transform: GPS → local cartesian ─────────────────────
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('gps/fix',           '/ublox_gps_node/fix'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps',      '/odometry/gps'),
                ('gps/filtered',      '/gps/filtered'),
            ],
        ),

        # ── GPS covariance inflator (for local EKF only) ─────────────────
        # Multiplies GPS odometry covariance by gps_scale before the local EKF
        # sees it, turning GPS from a hard position constraint into a soft
        # drift leash that lets the process model + LIO dominate.
        Node(
            package='roburoc_localization',
            executable='gps_covariance_inflator.py',
            name='gps_covariance_inflator',
            output='screen',
            parameters=[{'scale': gps_scale, 'use_sim_time': use_sim_time}],
            remappings=[
                ('input',  '/odometry/gps'),
                ('output', '/odometry/gps/inflated'),
            ],
        ),

        # ── Local EKF: odom → base_link ──────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/local'),
            ],
        ),

        # ── Global EKF: map → odom ───────────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
            ],
        ),

    ])
