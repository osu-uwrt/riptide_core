cmake_minimum_required(VERSION 3.8)
project(riptide_descriptions2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes robots urdf 
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
