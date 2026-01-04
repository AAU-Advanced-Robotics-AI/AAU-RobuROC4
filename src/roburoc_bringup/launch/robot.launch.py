"""
Bringup launch file for RobuROC4 real robot hardware.

Launches:
- Robot state publisher (TF transforms from URDF)
- CANopen driver (motor interface via PCAN)
- Robot controller (cmd_vel to motor commands)
- Joy node (gamepad input)
- IMU publisher (orientation data)

Usage:
    ros2 launch roburoc_bringup robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package paths
    pkg_description = get_package_share_directory('roburoc_description')
    pkg_bringup = get_package_share_directory('roburoc_bringup')

    # Process XACRO to URDF
    xacro_file = os.path.join(pkg_description, 'urdf', 'roburoc.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot state publisher - publishes TF transforms from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # CANopen driver - interfaces with motor controllers
        Node(
            package='roburoc_canopen',
            executable='robuROC_CANOpen.py',
            name='ROC_CAN',
            output='screen',
            parameters=[
                {'bustype': 'pcan'},
                {'channel': 'PCAN_USBBUS1'},
                {'bitrate': 1000000}
            ]
        ),

        # Robot controller - converts cmd_vel to motor commands
        Node(
            package='roburoc_controller',
            executable='robuROC_CTRL.py',
            name='ROC_CTRL',
            output='screen'
        ),

        # Joy node - gamepad input
        Node(
            package='joy',
            executable='joy_node',
            name='ROC_PAD',
            output='screen'
        ),

        # Include sensors launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'sensors.launch.py')
            )
        ),
    ])
