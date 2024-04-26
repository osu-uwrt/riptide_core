cmake_minimum_required(VERSION 3.8)
project(riptide_acoustics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(run_acoustics src/Acoustics.cpp)

target_include_directories(run_acoustics  PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(run_acoustics
std_msgs 
geometry_msgs
rclcpp
tf2
tf2_ros)

install(TARGETS run_acoustics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
