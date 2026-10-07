cmake_minimum_required(VERSION 3.8)
project(riptide_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Xml)
find_package(Qt5 COMPONENTS Charts QUIET)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(rclcpp_action REQUIRED)

add_subdirectory(vnproglib-1.2.0.0/cpp)

add_library(vn_config_core
  src/config_tool/calibration.cpp
  src/config_tool/configuration.cpp)
target_include_directories(vn_config_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vn_config_core PUBLIC Eigen3::Eigen Qt5::Core Qt5::Xml)

add_executable(imu src/driver/main.cpp)
target_include_directories(imu PRIVATE vnproglib-1.2.0.0/cpp/include include)
ament_target_dependencies(imu rclcpp std_msgs sensor_msgs geometry_msgs
  tf2_geometry_msgs riptide_msgs2 rclcpp_action)
target_link_libraries(imu libvncxx)

add_library(vn_config_ui
  src/config_tool/main_window.cpp
  src/config_tool/dashboard_widgets.cpp
  src/config_tool/sensor_transport.cpp
  include/riptide_imu/config_tool/main_window.hpp
  include/riptide_imu/config_tool/dashboard_widgets.hpp
  include/riptide_imu/config_tool/sensor_transport.hpp)
target_include_directories(vn_config_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vnproglib-1.2.0.0/cpp/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vn_config_ui PUBLIC vn_config_core libvncxx Qt5::Widgets)
if(TARGET Qt5::Charts)
  target_link_libraries(vn_config_ui PUBLIC Qt5::Charts)
endif()

add_executable(sensor_config src/config_tool/sensor_config_main.cpp)
target_include_directories(sensor_config PRIVATE include)
ament_target_dependencies(sensor_config rclcpp ament_index_cpp)
target_link_libraries(sensor_config vn_config_ui)

install(TARGETS imu sensor_config libvncxx vn_config_core vn_config_ui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README_sensor_config.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/hardware_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vn_hardware_acceptance)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Test)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # These whole-tree checks cannot exclude the bundled legacy VNProglib.
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # The installed checker tries to download the ROS package schema.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  target_link_libraries(test_calibration vn_config_core)
  ament_add_gtest(test_configuration test/test_configuration.cpp)
  target_link_libraries(test_configuration vn_config_core Qt5::Core Qt5::Xml)
  add_executable(test_dashboard test/test_dashboard.cpp)
  target_include_directories(test_dashboard PRIVATE include)
  target_compile_definitions(test_dashboard PRIVATE
    TEST_SHARE_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_dashboard vn_config_ui Qt5::Test)
  add_test(NAME test_dashboard COMMAND test_dashboard -platform offscreen)
endif()

ament_export_include_directories(include)
ament_export_libraries(vn_config_core)
ament_package()
