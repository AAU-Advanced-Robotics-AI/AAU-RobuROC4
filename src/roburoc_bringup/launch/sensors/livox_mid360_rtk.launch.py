"""
Sensor driver launch — configuration: livox_mid360_rtk

Starts the hardware drivers for:
  - Livox MID360 solid-state LiDAR  (→ /livox/lidar, /livox/imu)
  - u-blox ZED-F9P RTK-GPS receiver (→ /ublox_gps_node/fix, …)
  - NTRIP client (streams RTCM corrections from the Centipede network to the
    receiver so it can achieve RTK-float / RTK-fixed accuracy)

This file is selected automatically by robot.launch.py when
    sensor_config:=livox_mid360_rtk

NTRIP defaults target the public Centipede network (centipede/centipede) at
mount point HEGA (Aalborg area).  Override at launch time if needed:
    sensor_config:=livox_mid360_rtk \\
    ntrip_host:=crtk.net ntrip_mountpoint:=HEGA

Usage (direct):
    ros2 launch roburoc_bringup sensors/livox_mid360_rtk.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ublox_config = os.path.join(
        get_package_share_directory('ublox_gps'),
        'config',
        'zed_f9p.yaml',
    )

    return LaunchDescription([

        # ── NTRIP connection arguments ────────────────────────────────────
        DeclareLaunchArgument('ntrip_host',       default_value='crtk.net',
                              description='NTRIP caster hostname'),
        DeclareLaunchArgument('ntrip_port',       default_value='2101',
                              description='NTRIP caster port'),
        DeclareLaunchArgument('ntrip_mountpoint', default_value='OVTA',
                              description='NTRIP mountpoint (Aalborg area)'),
        DeclareLaunchArgument('ntrip_version',    default_value='RTCM3',
                              description='NTRIP protocol version'),
        DeclareLaunchArgument('ntrip_username',   default_value='centipede',
                              description='NTRIP username (Centipede public network)'),
        DeclareLaunchArgument('ntrip_password',   default_value='centipede',
                              description='NTRIP password (Centipede public network)'),
        DeclareLaunchArgument('ntrip_debug',      default_value='false',
                              description='Enable verbose NTRIP debug output'),

        # ── Livox MID360 driver ───────────────────────────────────────────
        # Publishes: /livox/lidar  (CustomMsg or PointCloud2 depending on xfer_format)
        #            /livox/imu   (sensor_msgs/Imu, built-in IMU at ~200 Hz)
        # Frame ID:  livox_frame  (matches URDF link in livox_mid360_rtk.urdf.xacro)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('livox_ros_driver2'),
                    'launch_ROS2',
                    'msg_MID360_launch.py',
                ])
            ])
        ),

        # ── u-blox ZED-F9P RTK-GPS ────────────────────────────────────────
        # Publishes: /ublox_gps_node/fix          (NavSatFix)
        #            /ublox_gps_node/fix_velocity  (TwistWithCovarianceStamped)
        #            /ublox_gps_node/navpvt        (ublox_msgs/NavPVT — RTK fix type)
        # Frame ID:  gps  (must match receiver's frame_id in zed_f9p.yaml, and the URDF link)
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[ublox_config],
        ),

        # ── NTRIP client (feeds RTCM corrections to the ZED-F9P) ─────────
        # Subscribes: /nmea (NMEA GGA sentences produced by the ublox driver)
        # Publishes:  /rtcm (rtcm_msgs/Message — RTCM3 correction stream)
        SetEnvironmentVariable(
            name='NTRIP_CLIENT_DEBUG',
            value=LaunchConfiguration('ntrip_debug'),
        ),
        Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            output='screen',
            parameters=[{
                'host':                          LaunchConfiguration('ntrip_host'),
                'port':                          LaunchConfiguration('ntrip_port'),
                'mountpoint':                    LaunchConfiguration('ntrip_mountpoint'),
                'ntrip_version':                 LaunchConfiguration('ntrip_version'),
                'authenticate':                  True,
                'username':                      LaunchConfiguration('ntrip_username'),
                'password':                      LaunchConfiguration('ntrip_password'),
                'ssl':                           False,
                'rtcm_frame_id':                 'gps',
                'nmea_max_length':               128,
                'nmea_min_length':               3,
                'rtcm_message_package':          'rtcm_msgs',
                'reconnect_attempt_max':         10,
                'reconnect_attempt_wait_seconds': 5,
                'rtcm_timeout_seconds':          4,
            }],
        ),

    ])
