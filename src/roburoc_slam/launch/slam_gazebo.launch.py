"""
Combined Gazebo + RTAB-Map SLAM Launch File

Launches both the Gazebo simulation and RTAB-Map SLAM in a single command.

Usage:
    ros2 launch roburoc_slam slam_gazebo.launch.py
    
    # Use teleop to drive and build map:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    return LaunchDescription([
        # Launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Include Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('roburoc_sim'), 'launch'),
                '/roburoc_gazebo_sim.launch.py'
            ])
        ),
        
        # Include RTAB-Map SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('roburoc_slam'), 'launch'),
                '/rtabmap_sim.launch.py'
            ])
        ),
    ])
