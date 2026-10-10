cmake_minimum_required(VERSION 3.16)
project(riptide_navigation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(TALOS_EKF_CODEGEN_ARCH "" CACHE STRING
  "Generated EKF architecture (x86_64 or arm64); inferred when empty")
if(NOT TALOS_EKF_CODEGEN_ARCH)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_LOWER)
  if(SYSTEM_PROCESSOR_LOWER MATCHES "^(aarch64|arm64)$")
    set(TALOS_EKF_CODEGEN_ARCH arm64)
  elseif(SYSTEM_PROCESSOR_LOWER MATCHES "^(x86_64|amd64)$")
    set(TALOS_EKF_CODEGEN_ARCH x86_64)
  else()
    message(FATAL_ERROR
      "Unsupported processor '${CMAKE_SYSTEM_PROCESSOR}'. Set "
      "TALOS_EKF_CODEGEN_ARCH to x86_64 or arm64 explicitly.")
  endif()
endif()
if(NOT TALOS_EKF_CODEGEN_ARCH MATCHES "^(x86_64|arm64)$")
  message(FATAL_ERROR
    "TALOS_EKF_CODEGEN_ARCH must be x86_64 or arm64, got "
    "'${TALOS_EKF_CODEGEN_ARCH}'")
endif()
message(STATUS "Using ${TALOS_EKF_CODEGEN_ARCH} generated Talos EKF core")
set(GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/matlab/generated/${TALOS_EKF_CODEGEN_ARCH}/talos_ekf_ert_rtw")
file(GLOB GENERATED_SOURCES CONFIGURE_DEPENDS "${GENERATED_DIR}/*.cpp")
list(FILTER GENERATED_SOURCES EXCLUDE REGEX "/ert_main\\.cpp$")
if(NOT GENERATED_SOURCES)
  message(FATAL_ERROR "Missing generated EKF sources. Run matlab/generate_talos_ekf.m")
endif()

add_library(talos_ekf_generated STATIC ${GENERATED_SOURCES})
target_include_directories(talos_ekf_generated PUBLIC
  "${GENERATED_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/riptide_navigation/codegen_compat")
target_compile_options(talos_ekf_generated PRIVATE -w)

add_executable(talos_ekf_node src/talos_ekf_node.cpp)
target_include_directories(talos_ekf_node PRIVATE include)
target_link_libraries(talos_ekf_node talos_ekf_generated)
ament_target_dependencies(talos_ekf_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp robot_localization sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS talos_ekf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY matlab DESTINATION share/${PROJECT_NAME}
  PATTERN "slprj" EXCLUDE
  PATTERN "html" EXCLUDE
  PATTERN "tmwinternal" EXCLUDE
  PATTERN "*.autosave" EXCLUDE
  PATTERN "*.slxc" EXCLUDE
  PATTERN "*.mexa64" EXCLUDE
  PATTERN "*.mat" EXCLUDE
  PATTERN "*.dmr" EXCLUDE
  PATTERN "*.mk" EXCLUDE
  PATTERN "rtw_proj.tmw" EXCLUDE
  PATTERN "ert_main.cpp" EXCLUDE)

ament_package()
