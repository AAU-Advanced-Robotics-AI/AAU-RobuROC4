# RobuROC4 - Autonomous Mobile Robot Platform

[![ROS 2 Build](https://github.com/simonbogh/P9-RobuROC4/actions/workflows/build.yaml/badge.svg)](https://github.com/simonbogh/P9-RobuROC4/actions/workflows/build.yaml)

ROS 2 software stack for the RobuROC4 robotic platform, featuring SLAM, CANopen motor control, and simulation capabilities.

## Table of Contents
- [Overview](#overview)
- [File Structure](#file-structure)
- [Mapping Algorithms](#mapping-algorithms)
- [Setup](#setup)

## Overview

This repository contains ROS 2 packages for operating and simulating the RobuROC4 platform. The RobuROC4 is a 4-wheeled skid-steer robot equipped with RGB-D cameras and LiDAR sensors for autonomous navigation.

**Key Features:**
- **RTAB-Map SLAM** — Real-time mapping and localization with loop closure detection
- **CANopen motor control** — Direct communication with wheel motor controllers via CAN bus
- **Gazebo simulation** — Full robot simulation with sensor plugins (RealSense, Velodyne)
- **Joystick control** — Manual teleoperation via gamepad controller

**Sensors Supported:**
- Intel RealSense RGB-D cameras (x2)
- Velodyne VLP-16 LiDAR
- IMU

## File Structure

```
P9-RobuROC4/
├── src/
│   ├── roburoc_sim/         # Simulation and SLAM launch files, URDF models
│   ├── roburoc_canopen/     # CANopen motor controller interface
│   ├── roburoc_controller/  # Joystick/controller input handling
│   ├── roburoc_canopen_interfaces/  # Custom CANopen ROS messages/services
│   ├── roburoc_imu_publisher/       # IMU data publishing node
│   └── thirdparty/          # External dependencies
│       ├── realsense_gazebo_plugin/
│       └── velodyne_simulator/
├── RobuROC 4 Docs/          # Platform documentation and manuals
└── Minimally working example/
```

## Mapping Algorithms

Two SLAM configurations are available depending on sensor preference:

| Launch File | Odometry Source | Mapping Sensors |
|-------------|-----------------|-----------------|
| `1Mapping.launch.py` | Visual (RGB-D) | Dual RGB-D cameras |
| `2Mapping.launch.py` | ICP (LiDAR) | Velodyne LiDAR |

Both configurations output a 3D occupancy grid map and robot pose estimates, visualized in RViz.

## Setup

### Prerequisites
- ROS 2 (Humble/Iron)
- Gazebo (for simulation)
- RTAB-Map ROS packages
- `teleop_twist_keyboard` (for simulation control)

### Running on Hardware

1. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

2. **Launch SLAM** (choose one based on sensor setup):
   ```bash
   ros2 launch roburoc_sim 1Mapping.launch.py  # RGB-D based
   ros2 launch roburoc_sim 2Mapping.launch.py  # LiDAR based
   ```

3. **Enable controller** (new terminal):
   ```bash
   ros2 launch roburoc_controller RobuROC_Controller_launch.py
   ```

4. **Visualize** — RViz launches automatically showing the map, point clouds, and robot pose.

### Running Simulation

1. **Build and launch Gazebo:**
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch roburoc_sim roburoc_sim.launch.py
   ```

2. **Control the robot** (new terminal):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Use `i/j/k/l` keys to drive the robot in Gazebo.