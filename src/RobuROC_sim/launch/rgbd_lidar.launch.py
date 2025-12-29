# Program dedicated to launch 2 realsense cameras. D405 were tested!
# (Please note that this program can manage maximum 4 depth cameras)

# Program modified by: Adrian Ricardez (https://github.com/adricort)
# Date: 07.07.2023
# Deutsches Zentrum für Luft- und Raumfahrt

# Requirements:

# Be sure that you did the build on your rtabmap workspace with the -DRTABMAP_SYNC_MULTI_RGBD=ON parameter.

# Launching the 2 realsense cameras (change your serial numbers):
#   $ ros2 launch realsense2_camera rs_multi_camera_launch.py pointcloud.enable1:=true pointcloud.enable2:=true filters:=colorizer align_depth:=true serial_no1:=_128422271521 serial_no2:=_128422272518

# Running the static publishers depending on the position of your cameras:
#   $ ros2 run tf2_ros static_transform_publisher --x 0.039 --y 0 --z 0 --yaw 0 --pitch 0 --roll 1.5708 --frame-id base_link --child-frame-id camera1_link
#   $ ros2 run tf2_ros static_transform_publisher --x -0 --y 0 --z 0.02 --yaw 1.5708 --pitch -1.5708 --roll 0 --frame-id base_link --child-frame-id camera2_link

#   $ ros2 launch rtabmap_examples rtabmap_D405x2.launch.py

# You should be able to visualize now, with the right rviz config, the camera's SLAM
# Have fun!

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


            # Nodes to launch
    Driver_vel = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('velodyne_driver'), 'launch'),
                '/velodyne_driver_node-VLP16-launch.py']),
        )
    Pointcloud_vel = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('velodyne_pointcloud'), 'launch'),
                '/velodyne_transform_node-VLP16-launch.py']),
        )

    # Lidar
    scan_cloud =launch_ros.actions.Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
              'deskewing':False,
              'use_sim_time':False,
              # RTAB-Map's internal parameters are strings:
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/VoxelSize': '0.1',
              'Icp/Epsilon': '0.001',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/MaxTranslation': '2',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.01',
              'Odom/ScanKeyFrameThr': '0.4',
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '15000',
              'OdomF2M/BundleAdjustment': 'false'
            }],
            remappings=[
              ('scan_cloud', '/velodyne_points'),
              ('scan', 'dummy1'),

              ('odom', 'odom'),

            ])
    
    # SLAM
    slam_node = launch_ros.actions.Node(
        package='rtabmap_slam', executable='rtabmap', output="screen",
        parameters=[{
            "rgbd_cameras": 2,
            "subscribe_depth": False,

            "subscribe_rgbd": False,
            "subscribe_rgb": False,
            'subscribe_scan_cloud':True,
            # "subscribe_odom_info": False,

            'odom_frame_id': 'odom',
            "frame_id": 'base_link',
            "map_frame_id": 'map',
            "publish_tf": True,
            "database_path": '~/.ros/rtabmap.db',
            "approx_sync": True,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "true"
        }],
        remappings=[


            ("odom", 'odom'),
            ('scan_cloud', '/velodyne_points')],
        arguments=["--delete_db_on_start"],
        prefix='',
        namespace='rtabmap'
    )



    return launch.LaunchDescription(
        [
            Driver_vel,
            Pointcloud_vel,
            scan_cloud,
            slam_node,

        ]
    )