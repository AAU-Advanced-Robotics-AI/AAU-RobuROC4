# RobuROC4 - Autonomous Mobile Robot Platform

[![ROS 2 Build](https://github.com/simonbogh/P9-RobuROC4/actions/workflows/build.yaml/badge.svg)](https://github.com/simonbogh/P9-RobuROC4/actions/workflows/build.yaml)

ROS 2 software stack for the RobuROC4 robotic platform, featuring SLAM, CANopen motor control, and simulation capabilities.

## Table of Contents
- [Overview](#overview)
- [File Structure](#file-structure)
- [Mapping Algorithms](#mapping-algorithms)
- [Setup](#setup)
  - [Running on Hardware](#running-on-hardware)
  - [Running Simulation](#running-simulation)
- [License](#license)
- [Contributing](#contributing)

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
│   ├── roburoc_bringup/          # Hardware bringup launch files
│   ├── roburoc_description/      # Robot URDF, meshes, and RViz config
│   ├── roburoc_sim/              # Simulation launch files
│   ├── roburoc_slam/             # SLAM configuration and launch files
│   ├── roburoc_canopen/          # CANopen motor controller interface
│   ├── roburoc_controller/       # Joystick/controller input handling
│   ├── roburoc_canopen_interfaces/  # Custom CANopen ROS messages/services
│   └── roburoc_imu_publisher/    # IMU data publishing node
├── RobuROC 4 Docs/               # Platform documentation and manuals
└── Minimally working example/
```

## Mapping Algorithms

SLAM configurations available for hardware and simulation:

**Hardware:**
| Launch File | Odometry Source | Mapping Sensors |
|-------------|-----------------|-----------------|
| `rtab_dual_simple.launch.py` | Visual (RGB-D) | Dual RGB-D cameras |
| `rgbd_lidar.launch.py` | Visual + ICP | RGB-D + Velodyne LiDAR |

**Simulation:**
| Launch File | Odometry Source | Mapping Sensors |
|-------------|-----------------|-----------------|
| `sim_1Mapping.launch.py` | Visual (RGB-D) | Dual RGB-D cameras |
| `sim_2Mapping.launch.py` | ICP (LiDAR) | Velodyne LiDAR |

All configurations output a 3D occupancy grid map and robot pose estimates, visualized in RViz.

## Setup

### Prerequisites
- ROS 2 Jazzy (Ubuntu 24.04) or ROS 2 Humble (Ubuntu 22.04)
- Gazebo (for simulation)
- RTAB-Map ROS packages
- Velodyne ROS packages
- Intel RealSense ROS packages

### Running on Hardware

1. **Install dependencies:**
   
   For ROS 2 Jazzy (Ubuntu 24.04):
   ```bash
   rosdep install -i --from-path src --rosdistro jazzy -y
   pip3 install --break-system-packages canopen pydantic nest-asyncio
   ```
   
   For ROS 2 Humble (Ubuntu 22.04):
   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   pip3 install canopen pydantic nest-asyncio
   ```

2. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Launch the robot:**
   ```bash
   ros2 launch roburoc_bringup robot.launch.py
   ```
   This launches the robot state publisher, CANopen driver, controller, joystick, and all sensors (IMU, Velodyne LiDAR, RealSense cameras).

   To launch only the sensors separately:
   ```bash
   ros2 launch roburoc_bringup sensors.launch.py
   ```

4. **Launch SLAM** (choose one based on sensor setup):
   ```bash
   ros2 launch roburoc_sim rtab_dual_simple.launch.py  # Dual RGB-D cameras
   ros2 launch roburoc_sim rgbd_lidar.launch.py        # RGB-D + LiDAR
   ```

5. **Visualize** — RViz launches automatically showing the map, point clouds, and robot pose.

### Running Simulation

1. **Build and launch Gazebo:**
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch roburoc_sim roburoc_gazebo_sim.launch.py
   ```

2. **Launch SLAM** (in a new terminal, choose one):
   ```bash
   ros2 launch roburoc_sim sim_1Mapping.launch.py  # RGB-D based
   ros2 launch roburoc_sim sim_2Mapping.launch.py  # LiDAR based
   ```

3. **Control the robot** (in a new terminal):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Use `i/j/k/l` keys to drive the robot in Gazebo.

## License

This project is licensed under multiple licenses:
- Most packages: MIT License
- `roburoc_canopen_interfaces`: Apache-2.0 License

See individual package.xml files for specific license information.

## Contributing

Contributions are welcome! For bug reports or feature requests, please open an issue on GitHub. For code contributions, please submit a pull request with a clear description of the changes.
