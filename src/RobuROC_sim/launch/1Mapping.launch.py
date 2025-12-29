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
    
    # Path to rviz config
    my_base_path = 'src/RobuROC_sim/src/rviz'   #path to all config files
    my_rviz_path = my_base_path+'/RobuROC_vis.rviz'       #config file for rviz



    # Path to model descriptiom
    modelFileRelativePath = 'description/RobuROC_model.urdf.xacro'
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()


    # Configure the nodes

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
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(my_rviz_path)]
    )

    realsense_dual = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(namePackage),'launch','dual_camera.launch.py'

            )]), launch_arguments={'serial_no1':"'310522072602'",                                             # old : 03442207065
                                   'camera_name':'camera1',
                                   'camera_namespace':'camera1',
                                   'serial_no2':"'336222071339'",                                            # old : 829212072207
                                   'camera_name':'camera2',
                                   'camera_namespace':'camera2'}.items()


    )
     # front 310522072602
    # back 336222071339


    rtab_dual_simple = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(namePackage), 'launch', 'rtab_dual_simple.launch.py'
        )]),
    )

    # Launch the nodes
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),        
        
        node_robot_state_publisher,
        joint_state_publisher,
        rviz,
        realsense_dual,            # launching both realsense cameras
        rtab_dual_simple,            # 1st mapping algorithm camera odometry and mapping
    ])