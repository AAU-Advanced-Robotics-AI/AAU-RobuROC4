cmake_minimum_required(VERSION 3.16)
project(lidar_gps_pgo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ublox_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration)
find_package(GTSAM REQUIRED)

# ---------------------------------------------------------- core (ROS-free)
add_library(pgo_core
  src/geo_utils.cpp
  src/scan_context.cpp
  src/pgo_backend.cpp
)
target_include_directories(pgo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pgo_core
  Eigen3::Eigen
  gtsam
  ${PCL_LIBRARIES}
)

# ------------------------------------------------------------------- node
add_executable(pgo_node src/pgo_node.cpp)
target_link_libraries(pgo_node pgo_core)
ament_target_dependencies(pgo_node
  rclcpp nav_msgs sensor_msgs geometry_msgs visualization_msgs std_srvs
  tf2_ros message_filters pcl_conversions ublox_msgs
)

install(TARGETS pgo_node pgo_core
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------------ tests
if(BUILD_TESTING)
  add_executable(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend pgo_core)
  install(TARGETS test_backend RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
