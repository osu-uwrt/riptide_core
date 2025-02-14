cmake_minimum_required(VERSION 3.5)
project(riptide_acoustics)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
  include
)

add_executable(localization
  src/localization_node.cpp
)

ament_target_dependencies(localization
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
)

install(TARGETS
  localization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch*"
)

install(PROGRAMS
  src/test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the test CSV files
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.csv"
)


ament_package()
