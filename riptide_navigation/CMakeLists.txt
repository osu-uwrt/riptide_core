cmake_minimum_required(VERSION 3.16)
project(riptide_navigation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(riptide_simulink REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Supervisor of the generated talos_navigation model (models/talos_navigation)
add_executable(talos_navigation_overseer src/talos_navigation_overseer.cpp)
target_link_libraries(talos_navigation_overseer riptide_simulink::model)
ament_target_dependencies(talos_navigation_overseer
  diagnostic_msgs geometry_msgs rclcpp robot_localization std_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS talos_navigation_overseer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_config_schema
    COMMAND ${Python3_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test/test_config_schema.py
      -q -p no:cacheprovider)
  set_tests_properties(test_config_schema PROPERTIES ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()
