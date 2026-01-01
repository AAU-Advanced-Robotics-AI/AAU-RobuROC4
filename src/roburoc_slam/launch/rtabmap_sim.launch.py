"""
RTAB-Map SLAM Launch File for Gazebo Simulation

Launches RTAB-Map nodes for single RGB-D camera (camera1/front) SLAM.
Use this when Gazebo simulation is already running.

Usage:
    # First, start Gazebo simulation:
    ros2 launch roburoc_sim roburoc_gazebo_sim.launch.py
    
    # Then start RTAB-Map:
    ros2 launch roburoc_slam rtabmap_sim.launch.py
    
    # Use teleop to drive and build map:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Based on: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_color.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Shared parameters for all RTAB-Map nodes
    parameters = [{
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True,  # Gazebo may not have exact timestamp sync
        'queue_size': 30,
    }]
    
    # Topic remappings for camera1 (front-facing D435)
    # These match the topics from roburoc_gazebo_sim.launch.py ros_gz_bridge
    remappings = [
        ('rgb/image', '/camera1/image'),
        ('rgb/camera_info', '/camera1/camera_info'),
        ('depth/image', '/camera1/depth'),
    ]
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo'
        ),
        
        # Visual Odometry from RGB-D
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ),
        
        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],  # Delete database on start
        ),
        
        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ),
    ])
