"""
RobuROC Gazebo Simulation Launch File

Launches the RobuROC robot in Gazebo Harmonic simulation with:
  - Gazebo world (empty_world.sdf)
  - Robot model spawning
  - ROS-Gazebo bridge for topics (cmd_vel, odom, sensors, etc.)
  - Robot state publisher
  - RViz visualization

Usage:
    ros2 launch roburoc_sim roburoc_gazebo_sim.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # =========================================================================
    # Configuration
    # =========================================================================
    robotXacroName = 'RobuROC'  # Must have same name as in xacro file

    # Package names
    namePackage = 'roburoc_sim'
    descriptionPackage = 'roburoc_description'
    # RTABPackage = 'rtabmap_launch'  # Uncomment when using RTAB-Map
    # d435Package = 'realsense2_camera'  # Uncomment when using real cameras
    # PointcloudPackage = 'velodyne_pointcloud'  # Not needed for simulation
    # VelDriverPackage = 'velodyne_driver'  # Not needed for simulation

    # File paths
    modelFileRelativePath = 'urdf/roburoc.urdf.xacro'
    worldFileRelativePath = 'worlds/empty_world.sdf'  # Gazebo Harmonic SDF format

    # Resolve paths
    pkg_project = get_package_share_directory(namePackage)
    pathModelFile = os.path.join(get_package_share_directory(descriptionPackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    my_rviz_path = os.path.join(get_package_share_directory('roburoc_sim'), 'rviz', 'RobuROC_vis.rviz')

    # Process robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # =========================================================================
    # Gazebo Simulation
    # =========================================================================
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {pathWorldFile}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot using ros_gz_sim create node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robotXacroName,
            '-z', '0.5'  # Spawn slightly above ground
        ],
        output='screen'
    )

    # =========================================================================
    # ROS Nodes
    # =========================================================================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    # NOTE: joint_state_publisher is NOT used in simulation because Gazebo
    # publishes joint states via the ros_gz_bridge. Using both causes conflicts.
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    # )

    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    # )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(my_rviz_path)]
    )

    # =========================================================================
    # Optional Launch Includes (not currently used in LaunchDescription)
    # =========================================================================
    LIDAR = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', 'vel_16.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'deskewing': 'false'
        }.items()
    )

    camera_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', 'camera_1.launch.py')
        ])
    )

    realsense_dual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', 'dual_camera.launch.py')
        ]),
        launch_arguments={
            'serial_no1': "'034422070675'",
            'camera_name': 'camera1',
            'camera_namespace': 'camera1',
            'serial_no2': "'829212072207'",
            'camera_name': 'camera2',
            'camera_namespace': 'camera2'
        }.items()
    )
    # Old TF calibration values:
    # 'tf.translation.x': '-1.2',
    # 'tf.translation.y': '0.075',
    # 'tf.translation.z': '-0.4',
    # 'tf.rotation.yaw': '-180',
    # 'tf.rotation.pitch': '31.0',
    # 'tf.rotation.roll': '1.0'

    # =========================================================================
    # Velodyne (Real Hardware Only)
    # =========================================================================
    # Pointcloud = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('velodyne_pointcloud'),
    #                      'launch', 'velodyne_transform_node-VLP16-launch.py')
    #     ])
    # )

    # VelDriver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('velodyne_driver'),
    #                      'launch', 'velodyne_driver_node-VLP16-launch.py')
    #     ]),
    #     launch_arguments={}.items()
    # )

    # =========================================================================
    # RTAB-Map (Uncomment when rtabmap packages are installed)
    # =========================================================================
    # rtab_vis = Node(
    #     package='rtabmap_viz',
    #     executable='rtabmap_viz',
    #     output='screen',
    #     parameters=[{
    #         'frame_id': 'camera_link',
    #         'subscribe_depth': False,
    #         'subscribe_rgbd': True,
    #         'subscribe_odom_info': True,
    #         'rgbd_cameras': 2,
    #         'approx_sync': False
    #     }],
    #     remappings=[
    #         ("rgbd_image0", '/camera1/rgbd_image'),
    #         ("rgbd_image1", '/camera2/rgbd_image'),
    #     ]
    # )

    # realsense_rtab = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('rtabmap_launch'),
    #                      'launch', 'rtabmap.launch.py')
    #     ]),
    #     launch_arguments={
    #         'rtabmap_args': "--delete_db_on_start",
    #         'rgb_topic': 'camera1/camera1/color/image_raw',
    #         'depth_topic': 'camera1/camera1/depth/image_rect_raw',
    #         'camera_info': "camera1/camera1/color/camera_info",
    #         'frame_id': 'camera1_link',
    #         'use_sim_time': 'true',
    #         'approx_sync': 'true',
    #         'qos': '2',
    #         'queue_size': '30'
    #     }.items()
    # )

    rtab_lidar_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', 'rtab_lidar_rgbd.launch.py')
        ])
    )

    rtab_dual_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', 'rtab_dual_rgbd.launch.py')
        ])
    )

    rtab_dual_simple = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', 'rtab_dual_simple.launch.py')
        ]),
    )

    two_rgbd_and_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(namePackage), 'launch', '2rgbd_lidar.launch.py')
        ]),
    )

    # =========================================================================
    # ROS-Gazebo Bridge
    # =========================================================================
    # Bridge configuration for topic communication between Gazebo and ROS 2
    # Discover Gazebo topics with: gz topic -l
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock for sim time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity commands (bidirectional)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # LIDAR - point cloud from gpu_lidar
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Camera 1 topics
            '/camera1/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera1/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera1/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Camera 2 topics
            '/camera2/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera2/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera2/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            # Remap lidar to expected topic name
            ('/velodyne_points/points', '/velodyne_points'),
            # Remap camera depth images to expected names
            ('/camera1/depth_image', '/camera1/depth'),
            ('/camera2/depth_image', '/camera2/depth'),
        ],
        output='screen'
    )

    # =========================================================================
    # Static Transform Publishers for RTAB-Map Optical Frames
    # 
    # Gazebo publishes camera data with frame_id=camera1_color_frame which uses
    # ROS convention (+X forward, +Y left, +Z up). RTAB-Map and other vision
    # algorithms expect optical frame convention (+Z forward, +X right, +Y down).
    # 
    # These static transforms create optical frames that RTAB-Map can use.
    # Rotation: roll=-π/2, yaw=-π/2 converts ROS frame to optical frame.
    # =========================================================================
    camera1_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera1_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
            '--frame-id', 'camera1_color_frame',
            '--child-frame-id', 'camera1_color_optical_frame_gz'
        ],
        parameters=[{'use_sim_time': True}],
    )
    
    camera2_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera2_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
            '--frame-id', 'camera2_color_frame',
            '--child-frame-id', 'camera2_color_optical_frame_gz'
        ],
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Gazebo
        gz_sim_launch,
        spawn_entity,
        ros_gz_bridge,

        # ROS nodes
        node_robot_state_publisher,
        # joint_state_publisher,  # Not needed - Gazebo provides joint states
        rviz,
        
        # Optical frame transforms for SLAM
        camera1_optical_tf,
        camera2_optical_tf,
    ])
