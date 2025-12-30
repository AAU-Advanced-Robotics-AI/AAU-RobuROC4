FROM ros:humble

# Install build dependencies (ARM64 compatible)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-vision-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install canopen pydantic nest-asyncio

# Set up workspace
WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/

# Install rosdep dependencies (skip unavailable ones)
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace (skip packages with missing deps)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --packages-skip realsense_gazebo_plugin velodyne_gazebo_plugins \
    --cmake-args -DCMAKE_BUILD_TYPE=Release" || \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-up-to roburoc_canopen roburoc_controller canopen_interfaces imu_publisher \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source workspace on shell startup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
