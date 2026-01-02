"""
Cartographer 3D SLAM Launch File for Gazebo Simulation

Launches Google Cartographer for 3D SLAM using the VLP16 LiDAR.
Uses wheel odometry from Gazebo simulation.

Usage:
    # First, start Gazebo simulation:
    ros2 launch roburoc_sim roburoc_gazebo_sim.launch.py

    # Then start Cartographer:
    ros2 launch roburoc_slam cartographer_3d.launch.py

    # Drive the robot to build the map:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Topics:
    Subscribed:
        - /velodyne_points (sensor_msgs/PointCloud2): VLP16 3D LiDAR
        - /odom (nav_msgs/Odometry): Wheel odometry from Gazebo
    Published:
        - /map (nav_msgs/OccupancyGrid): 2D occupancy grid (for navigation)
        - /submap_list: List of 3D submaps
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    pkg_roburoc_slam = get_package_share_directory('roburoc_slam')
    
    # Configuration file path
    cartographer_config_dir = os.path.join(pkg_roburoc_slam, 'config')
    configuration_basename = 'cartographer_3d.lua'
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of the occupancy grid (meters/cell)'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'
        ),
        
        # Cartographer SLAM node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                # Map point cloud topic to Cartographer's expected name
                ('points2', '/velodyne_points'),
                # Use Gazebo wheel odometry
                ('odom', '/odom'),
            ],
        ),
        
        # Occupancy grid node (generates 2D map from 3D data)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec
            ],
        ),
    ])
