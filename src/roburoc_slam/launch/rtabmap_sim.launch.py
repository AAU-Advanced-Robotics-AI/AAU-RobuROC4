"""
RTAB-Map SLAM Launch File for Gazebo Simulation

Launches RTAB-Map nodes for single RGB-D camera (camera1/front) SLAM.
Uses wheel odometry from Gazebo (more reliable for simulation).

This launch includes a frame_id_republisher that changes the frame_id from
camera1_color_frame to camera1_color_optical_frame_gz for RTAB-Map.

Usage:
    # First, start Gazebo simulation:
    ros2 launch roburoc_sim roburoc_gazebo_sim.launch.py
    
    # Then start RTAB-Map:
    ros2 launch roburoc_slam rtabmap_sim.launch.py
    
    # Use teleop to drive and build map:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Based on: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_color.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Parameters for RTAB-Map SLAM node
    # Using Gazebo's wheel odometry instead of visual odometry for simulation
    rtabmap_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_odom_info': False,  # Using Gazebo wheel odom, not visual odom
        'approx_sync': True,
        'sync_queue_size': 30,
        'qos_image': 0,  # Best effort
        'qos_camera_info': 0,
        'qos_odom': 0,
        
        # === CRITICAL: Detection Rate ===
        # Default is 1 Hz which causes slow updates! Set to 0 for maximum speed.
        'Rtabmap/DetectionRate': '0',  # 0 = process as fast as possible
        
        # === Memory Management ===
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'Mem/ImageKept': 'false',      # Don't keep raw images in RAM (faster)
        'Mem/BinDataKept': 'true',
        
        # === Update Thresholds ===
        # Small thresholds to get good coverage without too many nodes
        'RGBD/LinearUpdate': '0.1',    # Update every 10cm
        'RGBD/AngularUpdate': '0.1',   # Update every ~6 degrees
        
        # === Neighbor Link Refining (CRITICAL for scan alignment) ===
        # Enable to properly align point clouds between consecutive frames
        'Odom/Strategy': '0',
        'Odom/ResetCountdown': '0',
        'RGBD/NeighborLinkRefining': 'true',   # Enable scan alignment!
        'RGBD/OptimizeMaxError': '3.0',        # Allow some error
        
        # === Loop Closure Settings ===
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/MaxLocalRetrieved': '2',
        'RGBD/LocalRadius': '5.0',
        
        # === Registration Strategy ===
        # 0=Visual, 1=ICP, 2=Visual+ICP
        # Visual+ICP provides best scan alignment
        'Reg/Strategy': '2',           # Visual + ICP for best alignment
        'Reg/Force3DoF': 'true',       # Constrain to 2D (ground robot)
        
        # === ICP Parameters (for scan alignment) ===
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/Iterations': '30',
        'Icp/PointToPlane': 'true',
        
        # === Visual Features (tuned for rotation handling) ===
        'Vis/MinInliers': '8',         # Lower threshold for more matches
        'Vis/MaxFeatures': '1000',     # More features = more robust during rotation
        'Vis/MaxDepth': '5.0',
        'Vis/FeatureType': '6',        # ORB (good rotation invariance)
        'Vis/CorFlowMaxLevel': '5',    # More pyramid levels for optical flow
        
        # === Keypoint Detection (for loop closure) ===
        'Kp/MaxFeatures': '500',
        'Kp/DetectorStrategy': '6',    # ORB (match Vis/FeatureType to avoid warning)
        
        # === Optimizer ===
        'Optimizer/Strategy': '1',     # g2o
        'Optimizer/Iterations': '20',
        'Optimizer/Slam2D': 'true',
        'Optimizer/Robust': 'true',
        
        # === Grid Map (for visualization) ===
        'Grid/FromDepth': 'true',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/RayTracing': 'true',
    }]
    
    # Parameters for visualization
    rtabmap_viz_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'sync_queue_size': 30,
    }]
    
    # Topic remappings for camera1 (front-facing D435)
    # RTAB-Map subscribes to topics with optical frame convention.
    # We use the _optical topics which are republished with correct frame_id.
    remappings = [
        ('rgb/image', '/camera1/image_optical'),
        ('rgb/camera_info', '/camera1/camera_info_optical'),
        ('depth/image', '/camera1/depth_optical'),
        ('odom', '/odom'),  # Use Gazebo wheel odometry
    ]
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo'
        ),
        
        # =========================================================================
        # Frame ID Republisher
        # 
        # Republishes camera topics with frame_id changed to the optical frame
        # (camera1_color_optical_frame_gz). This allows RViz to display the
        # original topics correctly while RTAB-Map gets the optical frame it needs.
        # =========================================================================
        Node(
            package='roburoc_slam',
            executable='frame_id_republisher.py',
            name='camera1_frame_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_image_topic': '/camera1/image',
                'input_depth_topic': '/camera1/depth',
                'input_camera_info_topic': '/camera1/camera_info',
                'output_image_topic': '/camera1/image_optical',
                'output_depth_topic': '/camera1/depth_optical',
                'output_camera_info_topic': '/camera1/camera_info_optical',
                'target_frame_id': 'camera1_color_optical_frame_gz',
            }],
        ),
        
        # NOTE: Visual odometry (rgbd_odometry) is NOT used in simulation
        # because Gazebo provides reliable wheel odometry via /odom topic.
        # For real robot, you may want to use rgbd_odometry or sensor fusion.
        
        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=rtabmap_parameters,
            remappings=remappings,
            arguments=['-d'],  # Delete database on start
        ),
        
        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=rtabmap_viz_parameters,
            remappings=remappings,
        ),
    ])
