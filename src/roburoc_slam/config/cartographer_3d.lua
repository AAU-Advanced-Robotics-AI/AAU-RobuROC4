-- Cartographer 3D SLAM Configuration for RobuROC4
-- Uses VLP16 3D LiDAR with wheel odometry from Gazebo simulation
--
-- Usage:
--   ros2 launch roburoc_slam cartographer_3d.launch.py
--
-- Reference: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- Frame IDs (must match robot URDF and Gazebo)
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  
  -- Use odometry from Gazebo (wheel encoder simulation)
  provide_odom_frame = false,  -- Gazebo provides odom->base_link
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  
  -- Sensor configuration
  use_nav_sat = false,
  use_landmarks = false,
  
  -- Single 3D LiDAR (VLP16)
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- One VLP16 LiDAR
  
  -- TF lookup and publishing
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- Range data sampling
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- Enable 3D SLAM
MAP_BUILDER.use_trajectory_builder_3d = true

-- 3D Trajectory Builder Configuration
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- Point cloud filtering (VLP16 specs: 0.9m - 130m range)
TRAJECTORY_BUILDER_3D.min_range = 0.9
TRAJECTORY_BUILDER_3D.max_range = 50.0  -- Limit for indoor/outdoor mix

-- Voxel filter for point cloud downsampling
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- High resolution point cloud (for local SLAM)
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.0

-- Low resolution point cloud (for global SLAM / loop closure)
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 50.0

-- Ceres scan matcher (local optimization)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false

-- Motion filter (reduce redundant data when robot is stationary)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004

-- Submap configuration
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160

-- Pose graph optimization (global SLAM)
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.min_score = 0.62

-- Global localization (loop closure)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options
