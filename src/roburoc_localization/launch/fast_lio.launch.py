"""
FAST-LIO launch — MID360 LiDAR-inertial odometry with frame anchoring

Starts FAST-LIO and two static TF publishers needed to anchor its internal
frame convention to the robot's TF tree:

  1. odom → camera_init  (static)
       FAST-LIO uses camera_init as its world / odometry origin.  To make
       that origin coincide with the position of livox_frame at boot-up
       (i.e. as if the robot started at the origin of odom), this transform
       must equal base_link → livox_frame from the URDF.
       Values are loaded from config/sensor_mount.yaml [livox_mid360].

  2. livox_frame → body  (static, identity)
       FAST-LIO calls the sensor / IMU frame "body".  The livox_ros_driver2
       already stamps point clouds with livox_frame, so a zero-offset static
       TF is enough to make the two frame names interchangeable.

FAST-LIO config note:
    publish.tf_en is false in mid360.yaml — FAST-LIO publishes /Odometry only.
    robot_localization (localization.launch.py) is responsible for the
    map -> odom → base_link TF.

Usage:
    ros2 launch roburoc_localization fast_lio.launch.py
    ros2 launch roburoc_localization fast_lio.launch.py rviz:=false
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_livox_mount() -> dict:
    """Return the livox_mid360 section from config/sensor_mount.yaml."""
    path = os.path.join(
        get_package_share_directory('roburoc_localization'),
        'config', 'sensor_mount.yaml',
    )
    with open(path) as f:
        return yaml.safe_load(f)['livox_mid360']


# Resolved once at parse time — values come from sensor_mount.yaml.
_M = _load_livox_mount()


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')

    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_use     = LaunchConfiguration('rviz')
    rviz_cfg     = LaunchConfiguration('rviz_cfg')

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(fast_lio_share, 'config'),
            description='Directory containing the FAST-LIO YAML config file',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='mid360.yaml',
            description='FAST-LIO config file name (relative to config_path)',
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
            'rviz_cfg',
            default_value=os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz'),
            description='RViz config file path',
        ),

        # ── FAST-LIO (via upstream mapping.launch.py) ───────────────────
        # Subscribes: /livox/lidar  (CustomMsg)
        #             /livox/imu   (sensor_msgs/Imu)
        # Publishes:  /Odometry    (nav_msgs/Odometry, camera_init → body)
        #             /cloud_registered  (PointCloud2, world frame)
        # tf_en: false in mid360.yaml — TF is handled externally by the
        #         two static TF publishers below.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
            ),
            launch_arguments={
                'config_path':  config_path,
                'config_file':  config_file,
                'use_sim_time': use_sim_time,
                'rviz':         rviz_use,
                'rviz_cfg':     rviz_cfg,
            }.items(),
        ),


        # ── Static TF 1: odom → camera_init ─────────────────────────────
        # Places camera_init (FAST-LIO's world-frame origin) at the location
        # of livox_frame on the robot at boot-up.
        # Values loaded from config/sensor_mount.yaml [livox_mid360].
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_to_camera_init',
            output='screen',
            arguments=[
                '--x',     str(_M['x']),
                '--y',     str(_M['y']),
                '--z',     str(_M['z']),
                '--roll',  str(_M['roll']),
                '--pitch', str(_M['pitch']),
                '--yaw',   str(_M['yaw']),
                '--frame-id',       'odom',
                '--child-frame-id', 'camera_init',
            ],
        ),

        # ── Static TF 2: livox_frame → body  (identity) ──────────────────
        # FAST-LIO names the sensor/IMU frame "body".  livox_ros_driver2
        # publishes with frame_id livox_frame.  This zero-offset transform
        # makes the two names interchangeable without altering any data.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_livox_frame_to_body',
            output='screen',
            arguments=[
                '--x',     '0',
                '--y',     '0',
                '--z',     '0',
                '--roll',  '0',
                '--pitch', '0',
                '--yaw',   '0',
                '--frame-id',       'livox_frame',
                '--child-frame-id', 'body',
            ],
        ),
    ])
