cmake_minimum_required(VERSION 3.8)
project(roburoc_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gps_covariance_inflator.py
  scripts/gps_to_enu.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
