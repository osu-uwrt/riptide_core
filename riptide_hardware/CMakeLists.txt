cmake_minimum_required(VERSION 3.5)
project(riptide_hardware2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(rclpy REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nortek_dvl REQUIRED)
find_package(robot_localization REQUIRED)

# Include directories
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
  robot_localization
)

# Install the C++ executable
install(TARGETS
  depth_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Define console scripts for Python nodes
set(PYTHON_NODES
  "electrical_monitor"
  "firmware_monitor"
  "voltage_monitor"
  "sensor_monitor"
  "computer_monitor"
  "fake_ekf"
  "fake_dvl"
  "pose_converter"
  "rpm_echo"
  "tag_odom"
  "alternate_thruster"
  "imu_power_cycle"
  "simple_actuator_interface"
)

foreach(node ${PYTHON_NODES})
  # Create symbolic links without the .py extension
  install(CODE "
    execute_process(
      COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/riptide_hardware2/${node}.py ${CMAKE_CURRENT_BINARY_DIR}/${node}
    )
    execute_process(
      COMMAND chmod +x ${CMAKE_CURRENT_BINARY_DIR}/${node}
    )
  ")
endforeach()

# Install the symbolic links to the destination
install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/electrical_monitor
  ${CMAKE_CURRENT_BINARY_DIR}/firmware_monitor
  ${CMAKE_CURRENT_BINARY_DIR}/voltage_monitor
  ${CMAKE_CURRENT_BINARY_DIR}/sensor_monitor
  ${CMAKE_CURRENT_BINARY_DIR}/computer_monitor
  ${CMAKE_CURRENT_BINARY_DIR}/fake_ekf
  ${CMAKE_CURRENT_BINARY_DIR}/fake_dvl
  ${CMAKE_CURRENT_BINARY_DIR}/pose_converter
  ${CMAKE_CURRENT_BINARY_DIR}/rpm_echo
  ${CMAKE_CURRENT_BINARY_DIR}/tag_odom
  ${CMAKE_CURRENT_BINARY_DIR}/alternate_thruster
  ${CMAKE_CURRENT_BINARY_DIR}/imu_power_cycle
  ${CMAKE_CURRENT_BINARY_DIR}/simple_actuator_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch*"
)

# Install configuration files
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*"
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install marker file
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
