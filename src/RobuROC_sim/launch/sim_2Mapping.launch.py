import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    robotXacroName = 'RobuROC'  # Must have same name as in xacro file

    # Specify the name of the package and path to xacro file within the package
    namePackage = 'RobuROC_sim'
    RTABPackage = 'rtabmap_launch'
    d435Package = 'realsense2_camera'             
    PointcloudPackage = 'velodyne_pointcloud'
    VelDriverPackage = 'velodyne_driver'
    # Path to rviz config
    my_base_path = 'src/RobuROC_sim/src/rviz'   #path to all config files
    my_rviz_path = my_base_path+'/RobuROC_vis.rviz'       #config file for rviz

    # Use xacro to process the file
    # xacro_file = os.path.join(get_package_share_directory(namePackage),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()


    modelFileRelativePath = 'description/RobuROC_model.urdf.xacro'
    # worldFileRelativePath = 'worlds/RobuROC_env.world'
     # worldFileRelativePath = 'worlds/flat.world'
    worldFileRelativePath = 'worlds/cafe.world'
    # worldFileRelativePath = 'worlds/empty_world.world'
    # worldFileRelativePath = 'worlds/moon.world'

    pkg_project = get_package_share_directory(namePackage)

    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)

    pathWolrdFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))

    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world': pathWolrdFile}.items())


    # Configure the nodes

    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity',robotXacroName],
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # output='screen',
        output='screen',
        parameters=[{'robot_description':robotDescription,
        'use_sim_time': True}] # add other parameters here if required
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen', # add other parameters here if required
    )    
 
    rviz = Node(
            package='rviz2',
            # namespace='',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(my_rviz_path)]
    )


    lidar_odom_lidarrgbd_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(namePackage), 'launch', 'sim_rgbd_lidar.launch.py'
        )]),
    )

    # Launch the nodes
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),       
        gazeboLaunch,
        spawnModelNode, 
        node_robot_state_publisher,
        # joint_state_publisher,
        rviz,
        lidar_odom_lidarrgbd_map

    ])