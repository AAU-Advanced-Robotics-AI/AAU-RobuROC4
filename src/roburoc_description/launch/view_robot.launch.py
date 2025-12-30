"""
Launch file to visualize the RobuROC robot in RViz with interactive joint control.

Usage:
    ros2 launch roburoc_description view_robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package paths
    pkg_description = get_package_share_directory('roburoc_description')
    
    # Process XACRO to URDF
    xacro_file = os.path.join(pkg_description, 'urdf', 'RobuROC_model.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # RViz config
    rviz_config = os.path.join(pkg_description, 'rviz', 'view_robot.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot state publisher - publishes TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # Joint state publisher GUI - interactive sliders for joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])
