"""
Localization launch — FAST-LIO + Dual EKF + GPS/LIO Kabsch alignment

Fuses FAST-LIO2 LiDAR-inertial odometry with u-blox ZED-F9P RTK-GPS to
produce both local and global state estimates.

  lio_relay    /Odometry (FAST-LIO2) → /odometry/lio + /odometry/lio/global
  gps_to_enu   GPS fix → ENU Odometry → /odometry/gps/raw + /localization/datum
  lio_to_enu   Kabsch alignment → /localization/lio_to_enu
  gps_relay    lever-arm correction → /odometry/gps  |  velocity → /odometry/gps/local
  ekf_local    odom → base_link  (smooth LIO, published as /odom)
  ekf_global   GPS-anchored global pose; owns map→odom TF (publish_tf: true)

Data flow:
  /Odometry (FAST-LIO2)
      → lio_relay → /odometry/lio         → ekf_local  (differential)
                  → /odometry/lio/global   → ekf_global (absolute)
  /ublox_gps_node/fix
      → gps_to_enu → /odometry/gps/raw    → gps_relay (position input)
                   → /localization/datum   → lio_to_enu (ENU origin)
      → lio_to_enu → /localization/lio_to_enu → lio_relay (for /lio/global)
                                               → gps_relay (Kabsch theta)
      → gps_relay  → /odometry/gps         → ekf_global (lever-arm corrected)
  /ublox_gps_node/fix_velocity
      → gps_relay → /odometry/gps/local    → ekf_local  (body-frame velocity)

  lio_to_enu accumulates a sliding window of (LIO antenna, GPS antenna ENU)
  pairs and runs online Kabsch alignment to estimate the 2-D rigid transform
  T: odom→ENU.  ekf_global fuses /odometry/lio/global and /odometry/gps to
  produce the map→odom TF (publish_tf: true).

Cross-session route repeatability (datum):
  By default gps_to_enu auto-sets its ENU origin from the first RTK-quality fix.
  Pass datum_lat + datum_lon to pin the map frame to a fixed GPS coordinate
  so that routes recorded in one session replay at the same physical location
  in future sessions.  Edit config/field_datum.yaml and source it, or pass
  the args directly:
    ros2 launch roburoc_localization localization.launch.py \\
        datum_lat:=57.01234 datum_lon:=9.98765

Prerequisites:
  - robot.launch.py running (TF tree from URDF, sensor drivers)

Usage:
  ros2 launch roburoc_localization localization.launch.py
  ros2 launch roburoc_localization localization.launch.py rviz:=true
  ros2 launch roburoc_localization localization.launch.py datum_lat:=57.01234 datum_lon:=9.98765
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('roburoc_localization')
    fast_lio_launch  = os.path.join(pkg_localization, 'launch', 'fast_lio.launch.py')

    localization_config = LaunchConfiguration('localization_config')
    config_file = PathJoinSubstitution([
        FindPackageShare('roburoc_localization'), 'config', localization_config
    ])

    # Load lever-arm values at parse time from the single source of truth.
    mount_file = os.path.join(pkg_localization, 'config', 'sensor_mount.yaml')
    with open(mount_file) as f:
        _gps = yaml.safe_load(f)['gps_antenna']
    _lever_arm_x = float(_gps['lever_arm_x'])
    _lever_arm_y = float(_gps['lever_arm_y'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_use     = LaunchConfiguration('rviz')
    datum_lat    = LaunchConfiguration('datum_lat')
    datum_lon    = LaunchConfiguration('datum_lon')

    return LaunchDescription([

        DeclareLaunchArgument(
            'localization_config',
            default_value='debug_localization.yaml',
            description=(
                'YAML config file (filename only) for the dual EKF and lio_relay. '
                'Must be in roburoc_localization/config/. '
                'Use indoor_localization.yaml for LIO-only indoor testing.'
            ),
        ),

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
            'datum_lat',
            default_value='nan',
            description=(
                'Fixed datum latitude  (degrees) for cross-session route repeatability. '
                'Leave as nan to auto-set from the first GPS fix (same-session operation). '
                'See config/field_datum.yaml for how to set this permanently.'
            ),
        ),
        DeclareLaunchArgument(
            'datum_lon',
            default_value='nan',
            description=(
                'Fixed datum longitude (degrees) for cross-session route repeatability. '
                'Must be set together with datum_lat.'
            ),
        ),

        # ── FAST-LIO2 + static TFs ───────────────────────────────────────
        # Includes fast_lio.launch.py which adds two static TF publishers:
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
        # Publishes:  /odometry/lio         (odom → base_link, local EKF)
        #             /odometry/lio/global  (map  → base_link, global EKF)
        Node(
            package='roburoc_localization',
            executable='lio_relay.py',
            name='lio_relay',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),


        # ── GPS → ENU (antenna position) ──────────────────────────────────
        # Converts NavSatFix to flat-Earth ENU Odometry (raw antenna position,
        # no lever-arm correction).  The first RTK-quality fix becomes the ENU
        # datum / map-frame origin.
        # Publishes:
        #   /odometry/gps/raw     — GPS antenna pos in map frame (child=gps)
        #   /localization/datum   — datum NavSatFix (transient_local, once)
        Node(
            package='roburoc_localization',
            executable='gps_to_enu.py',
            name='gps_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'datum_lat':    datum_lat,
                'datum_lon':    datum_lon,
            }],
        ),

        # ── LIO / GPS Kabsch alignment ─────────────────────────────────────
        # Computes the optimal 2-D rigid transform T: odom→ENU using a sliding
        # window of (LIO antenna, GPS antenna ENU) pairs.  No TF is published —
        # ekf_global owns the map→odom TF (publish_tf: true).
        # Publishes:
        #   /localization/lio_to_enu  — Kabsch transform (transient_local)
        #   /odometry/gps             — lever-arm corrected base_link in map
        Node(
            package='roburoc_localization',
            executable='lio_to_enu.py',
            name='lio_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'lever_arm_x':  _lever_arm_x,
                'lever_arm_y':  _lever_arm_y,
                'datum_lat':    datum_lat,
                'datum_lon':    datum_lon,
            }],
        ),

        # ── GPS relay: position + velocity → base_link frame ─────────────────
        # Applies the lever-arm correction to /odometry/gps/raw (antenna
        # position) and publishes /odometry/gps (base_link, map frame).
        # Also rotates fix_velocity (ENU) into base_link and publishes
        # /odometry/gps/local.  Both only after Kabsch alignment is available.
        Node(
            package='roburoc_localization',
            executable='gps_relay.py',
            name='gps_relay',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'lever_arm_x':  float(_lever_arm_x),
                'lever_arm_y':  float(_lever_arm_y),
            }],
        ),

        # ── Local EKF: odom → base_link ──────────────────────────────────
        # Smooth, LIO-dominated estimate for control feedback.
        # Published as /odom (REP-105 reserved name).
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
        # GPS-anchored estimate used for path recording and replay.
        # Published as /odometry/filtered/global.
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

