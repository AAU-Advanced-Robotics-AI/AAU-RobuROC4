"""
Bringup launch file for the RobuROC4 real robot hardware.

Launches:
  - robot_state_publisher    (TF tree from URDF, sensor frames included)
  - roburoc_canopen          (motor interface via PCAN-USB)
  - roburoc_controller       (joy → cmd_vel → motor commands)
  - joy_node                 (gamepad / joystick input)
  - sensor driver stack      (selected by sensor_config, can be suppressed)

Sensor configurations
─────────────────────
The sensor_config argument selects both the URDF geometry (which frames are
published by robot_state_publisher) and the hardware driver stack to start.
Each configuration is a self-contained pair of files:

  realsense_velodyne  (default)
      urdf:    roburoc_description/urdf/sensors/realsense_velodyne.urdf.xacro
      drivers: roburoc_bringup/launch/sensors/realsense_velodyne.launch.py

  livox_mid360_rtk
      urdf:    roburoc_description/urdf/sensors/livox_mid360_rtk.urdf.xacro
      drivers: roburoc_bringup/launch/sensors/livox_mid360_rtk.launch.py

To add a new sensor configuration:
  1. Add urdf/sensors/<name>.urdf.xacro in roburoc_description (links + joints).
  2. Add a <xacro:if> block in roburoc.urdf.xacro.
  3. Add launch/sensors/<name>.launch.py in roburoc_bringup (hardware drivers).
  Then pass sensor_config:=<name> here.

Usage
─────
  ros2 launch roburoc_bringup robot.launch.py
  ros2 launch roburoc_bringup robot.launch.py sensor_config:=livox_mid360_rtk
  ros2 launch roburoc_bringup robot.launch.py enable_sensors:=false
  ros2 launch roburoc_bringup robot.launch.py max_speed:=0.5 turbo_speed:=1.0
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Launch setup (runs at launch time, after arguments are resolved) ──────────

def launch_setup(context, *args, **kwargs):
    sensor_config        = LaunchConfiguration('sensor_config').perform(context)
    use_sim_time         = LaunchConfiguration('use_sim_time').perform(context)
    enable_sensors       = LaunchConfiguration('enable_sensors').perform(context).lower()
    enable_joy           = LaunchConfiguration('enable_joy').perform(context).lower()
    max_speed            = LaunchConfiguration('max_speed').perform(context)
    turbo_speed          = LaunchConfiguration('turbo_speed').perform(context)
    navigation_mode      = LaunchConfiguration('navigation_mode').perform(context).lower()
    joy_management_topic = LaunchConfiguration('joy_management_topic').perform(context)

    pkg_description = get_package_share_directory('roburoc_description')
    pkg_bringup     = get_package_share_directory('roburoc_bringup')

    # Process the URDF with the chosen sensor configuration.
    # The sensor_config mapping drives the <xacro:if> blocks in roburoc.urdf.xacro,
    # which selects the correct sensor links and joints for robot_state_publisher.
    xacro_file = os.path.join(pkg_description, 'urdf', 'roburoc.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'sensor_config': sensor_config},
    ).toxml()

    nodes = [

        # Robot state publisher — publishes all static TF frames from the URDF,
        # including sensor mounting positions for the selected sensor_config.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time':      use_sim_time == 'true',
            }],
        ),

        # CANopen driver — interfaces with the four AMC motor controllers
        Node(
            package='roburoc_canopen',
            executable='robuROC_CANOpen.py',
            name='ROC_CAN',
            output='screen',
            parameters=[
                {'bustype': 'pcan'},
                {'channel': 'PCAN_USBBUS1'},
                {'bitrate': 1000000},
            ],
        ),

        # Robot controller — translates joy/cmd_vel to per-wheel velocity commands
        Node(
            package='roburoc_controller',
            executable='robuROC_CTRL.py',
            name='ROC_CTRL',
            output='screen',
            parameters=[{
                'max_speed':            float(max_speed),
                'turbo_speed':          float(turbo_speed),
                'navigation_mode':      navigation_mode == 'true',
                'joy_management_topic': joy_management_topic,
            }],
        ),

        # Joy node — gamepad / joystick input
        # Set enable_joy:=false when using the botany navigation stack, which
        # manages joystick input through its own joy_relay → control_mux pipeline.
        # (If joy_node runs with ROC_CTRL active, joystick "no-button" messages
        # continuously send zero velocity, blocking autonomous navigation.)
        *([Node(
            package='joy',
            executable='joy_node',
            name='ROC_PAD',
            output='screen',
        )] if enable_joy == 'true' else []),

    ]

    # Sensor driver stack — include the launch file that matches sensor_config.
    if enable_sensors == 'true':
        sensors_launch_path = os.path.join(
            pkg_bringup, 'launch', 'sensors', f'{sensor_config}.launch.py'
        )
        if not os.path.isfile(sensors_launch_path):
            raise RuntimeError(
                f"No sensor driver launch file found for sensor_config='{sensor_config}'.\n"
                f"Expected: {sensors_launch_path}\n"
                f"Create launch/sensors/{sensor_config}.launch.py in roburoc_bringup."
            )
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch_path),
            )
        )

    return nodes


# ── Launch description ────────────────────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'sensor_config',
            default_value='livox_mid360_rtk',
            description=(
                'Sensor suite to load.  Selects both the URDF frames and the '
                'hardware drivers.  Available: realsense_velodyne, livox_mid360_rtk.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock.',
        ),
        DeclareLaunchArgument(
            'enable_sensors',
            default_value='true',
            description='Start the sensor driver stack.  Set false to bring up only the drivetrain.',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Normal operating speed limit [m/s].',
        ),
        DeclareLaunchArgument(
            'navigation_mode',
            default_value='false',
            description=(
                'When true, ROC_CTRL handles only motor management (■ brake/enable, '
                '⬤ recover) from joy_management_topic. Movement commands come via '
                '/roburoc/cmd_vel from the navigation stack (joy_relay → control_mux). '
                'Set automatically to true by robot_real.launch.py.'
            ),
        ),
        DeclareLaunchArgument(
            'joy_management_topic',
            default_value='/joy_physical',
            description='Joy topic to watch for motor management buttons when navigation_mode=true.',
        ),
        DeclareLaunchArgument(
            'turbo_speed',
            default_value='2.0',
            description='Turbo mode speed limit [m/s] (activated by R2 trigger, max ~2.75 m/s).',
        ),
        DeclareLaunchArgument(
            'enable_joy',
            default_value='true',
            description=(
                'Start joy_node for direct joystick → ROC_CTRL control. '
                'Set false when using botany_ws navigation (joy is handled '
                'by joy_relay → control_mux in that stack instead).'
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
