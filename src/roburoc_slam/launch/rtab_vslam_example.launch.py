import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration variables:
    #localization = LaunchConfiguration('localization')
    #rtabmap_args = LaunchConfiguration('rtabmap_args')


    use_grayscale = LaunchConfiguration('use_grayscale')
    rgb_topic = LaunchConfiguration('rgb_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    config_file = os.path.join(
    FindPackageShare('gorm_vslam').find('gorm_vslam'),
    'config',
    'rtabmap_params.yaml'
    )
    
    #Open the config files
    with open(config_file,'r') as file:
        loaded_parameters = yaml.safe_load(file)
    #Standard ros2 config files are nested so we grab just the ros parameters to ensure we get the correct data structure
    launch_args = loaded_parameters["rtabmap"]["ros__parameters"]

    
    # Add flexible launch configurations
    launch_args['rviz'] = LaunchConfiguration('rviz')
    launch_args['rtabmap_viz'] = LaunchConfiguration('rtabmap_viz')
    #launch_args['localization'] = LaunchConfiguration('localization')



    # RTAB-Map launch directory
    rtabmap_launch_dir = FindPackageShare('rtabmap_launch').find('rtabmap_launch')

    map_db_path ='/home/roy/Documents/vslam_maps/inside_recording.db'

    stereo_namespace = '/zed_front/zed'
    left_image = f'{stereo_namespace}/left/image_rect_color'
    right_image = f'{stereo_namespace}/right/image_rect_color'
    left_info = f'{stereo_namespace}/left/camera_info'
    right_info = f'{stereo_namespace}/right/camera_info'

    map_republisher = Node(
    package='gorm_vslam',
    executable='map_republisher.py',
    name='map_republisher',
    output='screen'
    )


    # Include RTAB-Map launch file with conditional topics
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments=launch_args.items()
    )

    # Delay RTAB-Map launch to allow camera initialization
    delayed_actions = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='Starting Rtab-slam after ZED camera initialization delay...'),
        ]
    )

    pointcloud_converter = Node(
        package='elevation_map',
        executable='pointcloud_converter',
        name='pointcloud_converter',
        output='screen'
    )

    static_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_map_to_odom',
    arguments=[
        '--x', '1.0',        # translation (m)
        '--y', '0.0',
        '--z', '0.0',
        '--yaw', '0.0',      # rotation (radians)
        '--pitch', '0.0',
        '--roll', '0.0',
        '--frame-id', 'map',          # parent frame
        '--child-frame-id', 'odom'  # child frame
        ]
    )


    return LaunchDescription([
        LogInfo(msg='Starting ZED2i Camera with RTAB-Map VSLAM...'),

        # Declare launch arguments
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ (optional).'),
        #DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('use_grayscale', default_value='false', description='Use grayscale input images'),
       

        # Conditionally set topics based on grayscale flag
        DeclareLaunchArgument(
            'rgb_topic',
            default_value=PythonExpression([
                "\"/zed_front/zed/rgb_gray/image_rect_gray\" if '", use_grayscale, "' == 'true' else \"/zed_front/zed/rgb/image_rect_color\""
            ]),
            description='RGB image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=PythonExpression([
                "\"/zed_front/zed/rgb_gray/camera_info\" if '", use_grayscale, "' == 'true' else \"/zed_front/zed/rgb/camera_info\""
            ]),
            description='Camera info topic'
        ),

        #Conditionally set if rtabmap.db should be deleted. (we save this code but the functionality is move)
        # DeclareLaunchArgument(
        #     'rtabmap_args',
        #     default_value=PythonExpression([
        #         "\"--delete_db_on_start\" if '",localization, "'=='false' else ''"
        #     ]),
        #     description ='Deletes the previous map if localization is off'
        # ),

        
        # Print selected topics
        LogInfo(msg=PythonExpression([
            "'Selected RGB topic: ' + '", rgb_topic, "'"
        ])),
        LogInfo(msg=PythonExpression([
            "'Selected Camera Info topic: ' + '", camera_info_topic, "'"
        ])),

        #static_transform,
        rtabmap_launch,
        pointcloud_converter,
        map_republisher,
        delayed_actions
    ])