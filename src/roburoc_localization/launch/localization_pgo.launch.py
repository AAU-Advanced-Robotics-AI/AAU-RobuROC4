"""
localization_pgo.launch.py — online localization with the GTSAM PGO backend
instead of the global EKF.

Same skeleton as localization.launch.py, but the entire GPS-anchored global half
(gps_to_enu + lio_to_enu Kabsch + gps_relay + ekf_global) is replaced by a single
node, lidar_gps_pgo/pgo_node, which does its own ENU<->map alignment, RTK-class
GPS fusion, and ScanContext loop closures, and owns the map->odom TF.

  FAST-LIO2  ─ /Odometry ──────────► lio_relay ─ /odometry/lio ─► ekf_local ─► /odom
             │                                                     (odom->base_link TF)
             ├─ /Odometry + /cloud_registered ┐
  u-blox ────┴─ /fix + /navpvt ───────────────┴─► pgo_node ─► map->odom TF + /pgo/odometry

Roles vs the dual-EKF:
  ekf_local  (KEPT)      smooth LIO-based odom->base_link, published as /odom  (control)
  pgo_node   (REPLACES   GPS-anchored global pose; owns map->odom; /pgo/odometry
              ekf_global) at full LIO rate; re-corrects the past via loop closures

Prerequisites (robot): robot.launch.py running (URDF TF tree + sensor drivers).
On the robot, pgo_node reads livox_frame->gps_link / ->base_link from live TF; the
fallback extrinsics in pgo_online.yaml are used only if that lookup fails.

Usage:
  ros2 launch roburoc_localization localization_pgo.launch.py
  ros2 launch roburoc_localization localization_pgo.launch.py rviz:=true
  # bag replay / offline sim: see offline_pgo_online.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_loc = get_package_share_directory('roburoc_localization')
    fast_lio_launch = os.path.join(pkg_loc, 'launch', 'fast_lio.launch.py')

    localization_config = LaunchConfiguration('localization_config')
    ekf_cfg = PathJoinSubstitution(
        [FindPackageShare('roburoc_localization'), 'config', localization_config])

    pgo_cfg = LaunchConfiguration('pgo_config')
    pgo_cfg_path = PathJoinSubstitution(
        [FindPackageShare('lidar_gps_pgo'), 'config', pgo_cfg])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_use = LaunchConfiguration('rviz')
    pgo_save_dir = LaunchConfiguration('pgo_save_dir')

    return LaunchDescription([
        DeclareLaunchArgument('localization_config',
                              default_value='debug_localization.yaml',
                              description='config for lio_relay + ekf_local'),
        DeclareLaunchArgument('pgo_config', default_value='pgo_online.yaml',
                              description='pgo_node config (in lidar_gps_pgo/config)'),
        DeclareLaunchArgument('pgo_save_dir', default_value='~/pgo_output',
                              description='pgo_node save_directory (georeference/map/TUM)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # ── FAST-LIO2 + static TFs (odom->camera_init, livox_frame->body) ──────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'use_sim_time': use_sim_time,
                              'rviz': rviz_use}.items(),
        ),

        # ── LIO relay: /Odometry -> /odometry/lio (odom->base_link, for local EKF)
        Node(
            package='roburoc_localization', executable='lio_relay.py',
            name='lio_relay', output='screen',
            parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
        ),

        # ── Local EKF: smooth odom->base_link, published as /odom ──────────────
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_local', output='screen',
            parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
        ),

        # ── PGO: global localization, owns map->odom (replaces ekf_global) ─────
        Node(
            package='lidar_gps_pgo', executable='pgo_node',
            name='pgo', output='screen',
            parameters=[pgo_cfg_path,
                        {'use_sim_time': use_sim_time,
                         'save_directory': pgo_save_dir}],
        ),
    ])
