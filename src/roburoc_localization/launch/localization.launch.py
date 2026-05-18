"""
Localization launch — FAST-LIO + Dual EKF + gps_to_enu

Fuses FAST-LIO2 LiDAR-inertial odometry with u-blox ZED-F9P RTK-GPS to
produce both local and global state estimates.

  lio_relay    /Odometry (FAST-LIO2) → /odometry/lio + /odometry/lio/global
  gps_to_enu   GPS fix + velocity bearing → /odometry/gps/raw  (odom frame)
               Heading is estimated from GPS velocity at first motion —
               move straight forward briefly after startup.
  gps_relay    /odometry/gps/raw → /odometry/gps + /odometry/gps/local
  ekf_local    odom → base_link  (smooth LIO, published as /odom)
  ekf_global   map  → odom       (GPS-anchored, published as /odometry/filtered/global)

Data flow:
  /Odometry (FAST-LIO2)
      → lio_relay → /odometry/lio       → ekf_local  (differential)
                  → /odometry/lio/global → ekf_global (absolute)
  /ublox_gps_node/fix + /ublox_gps_node/fix_velocity
      → gps_to_enu → /odometry/gps/raw
      → gps_relay  → /odometry/gps       → ekf_global (tight anchor)
                   → /odometry/gps/local → ekf_local  (soft leash)

Cross-session route repeatability (datum):
  By default gps_to_enu auto-sets its map origin from the first GPS fix.
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

    use_sim_time     = LaunchConfiguration('use_sim_time')
    gps_scale_local  = LaunchConfiguration('gps_scale_local')
    gps_scale_global = LaunchConfiguration('gps_scale_global')
    rviz_use         = LaunchConfiguration('rviz')
    datum_lat        = LaunchConfiguration('datum_lat')
    datum_lon        = LaunchConfiguration('datum_lon')

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

        # ── GPS → local cartesian (replaces navsat_transform_node) ───────
        # Derives ENU→odom rotation from GPS velocity at first motion.
        # IMPORTANT: drive straight forward briefly after startup so the
        # bearing lock triggers before attempting any navigation.
        # Lever-arm values loaded from config/sensor_mount.yaml.
        # datum_lat/datum_lon: when set, pins the map origin to a fixed GPS
        # coordinate for cross-session route repeatability.
        Node(
            package='roburoc_localization',
            executable='gps_to_enu.py',
            name='gps_to_enu',
            output='screen',
            parameters=[{
                'use_sim_time':    use_sim_time,
                'speed_threshold': 0.3,
                'bearing_samples': 5,
                'bearing_timeout': 15.0,
                'lever_arm_x':     _lever_arm_x,
                'lever_arm_y':     _lever_arm_y,
                'odom_topic':      '/odometry/lio',
                'datum_lat':       datum_lat,
                'datum_lon':       datum_lon,
            }],
        ),

        # ── GPS relay: /odometry/gps/raw → two covariance-scaled streams ─
        #   /odometry/gps        — global EKF input (tight anchor, ×scale_global)
        #   /odometry/gps/local  — local EKF input  (soft drift leash, ×scale_local)
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

