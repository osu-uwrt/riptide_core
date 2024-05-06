cmake_minimum_required(VERSION 3.5)
project(riptide_hardware2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(riptide_msgs2 REQUIRED)

find_package(rclpy REQUIRED)  # For Python nodes
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nortek_dvl REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(robot_localization REQUIRED)
find_package(gpiod REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Add executable for the DepthConverter
add_executable(depth_converter src/depth_converter.cpp)
ament_target_dependencies(depth_converter
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  riptide_msgs2
  Eigen3
)

# Install the executable
install(TARGETS
  depth_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

# Install launch files, configurations, and other data
install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
