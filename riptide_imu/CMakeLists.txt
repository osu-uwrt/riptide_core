cmake_minimum_required(VERSION 3.8)
project(riptide_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#include(vnproglib-1.2.0.0/cpp/CMakeLists.txt)
add_subdirectory(vnproglib-1.2.0.0/cpp)

#Grab vnproglib header files
include_directories(vnproglib-1.2.0.0/cpp/include)

#Build executable
add_executable(imu src/main.cpp)

#Define a variable with all dependencies
set(deps 
  rclcpp
  std_msgs
  sensor_msgs
)

# Link deps
ament_target_dependencies(imu ${deps})

#Link vnproglib
target_link_libraries(imu
  libvncxx
)

#Install executables
install(TARGETS imu
  DESTINATION lib/${PROJECT_NAME}
)

#Install vnproglib
install(TARGETS libvncxx
  DESTINATION share/${PROJECT_NAME}
)

#Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
