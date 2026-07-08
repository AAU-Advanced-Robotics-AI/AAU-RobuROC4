"""
Sensor driver launch — configuration: realsense_velodyne

Starts the hardware drivers for:
  - RobuROC IMU publisher (chassis IMU → /imu topic)
  - Velodyne VLP-16 LiDAR driver  (→ /velodyne_points)
  - Velodyne pointcloud transform  (converts raw packets to PointCloud2)
  - Two Intel RealSense D435i cameras (serial numbers stored below)

This file is selected automatically by robot.launch.py when
    sensor_config:=realsense_velodyne   (the default)

Serial numbers (check with: rs-enumerate-devices | grep Serial):
    camera1 (front)  034422070675
    camera2 (rear)   829212072207

Usage (direct):
    ros2 launch roburoc_bringup sensors/realsense_velodyne.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        # Chassis IMU publisher
        Node(
            package='roburoc_imu_publisher',
            executable='roburoc_imu_publisher',
            name='imu_publisher',
            output='screen',
        ),

        # Velodyne VLP-16 driver (raw packet capture)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne_driver'),
                    'launch',
                    'velodyne_driver_node-VLP16-launch.py',
                ])
            ])
        ),

        # Velodyne pointcloud transform (raw packets → PointCloud2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne_pointcloud'),
                    'launch',
                    'velodyne_transform_node-VLP16-launch.py',
                ])
            ])
        ),

        # RealSense D435i — front (camera1) and rear (camera2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_dual_camera_launch.py',
                ])
            ]),
            launch_arguments={
                'serial_no1':        "'034422070675'",
                'camera_name1':      'camera1',
                'camera_namespace1': 'camera1',
                'serial_no2':        "'829212072207'",
                'camera_name2':      'camera2',
                'camera_namespace2': 'camera2',
            }.items(),
        ),

    ])
