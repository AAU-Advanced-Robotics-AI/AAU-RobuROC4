"""
Launch file for RobuROC4 sensors.

Launches:
- IMU publisher
- Velodyne VLP-16 LiDAR driver and pointcloud transform
- RealSense dual cameras

Usage:
    ros2 launch roburoc_bringup sensors.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # IMU publisher
        Node(
            package='roburoc_imu_publisher',
            executable='roburoc_imu_publisher',
            name='imu_publisher',
            output='screen'
        ),

        # Velodyne VLP-16 driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne_driver'),
                    'launch',
                    'velodyne_driver_node-VLP16-launch.py'
                ])
            ])
        ),

        # Velodyne pointcloud transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne_pointcloud'),
                    'launch',
                    'velodyne_transform_node-VLP16-launch.py'
                ])
            ])
        ),

        # RealSense dual cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_dual_camera_launch.py'
                ])
            ]),
            launch_arguments={
                'serial_no1': "'034422070675'",
                'camera_name1': 'camera1',
                'camera_namespace1': 'camera1',
                'serial_no2': "'829212072207'",
                'camera_name2': 'camera2',
                'camera_namespace2': 'camera2',
            }.items()
        ),
    ])
