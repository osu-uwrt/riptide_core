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
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
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
install(PROGRAMS
  scripts/analyze_navigation_bag.py
  scripts/record_navigation_run.sh
  scripts/verify_jetson_install.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY validation DESTINATION share/${PROJECT_NAME}
  PATTERN "runs" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_xmllint
  )
  file(GLOB_RECURSE GENERATED_LINT_EXCLUDES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/matlab/generated/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/matlab/generated/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/matlab/generated/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/matlab/generated/*.hpp"
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${GENERATED_LINT_EXCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/matlab/talos_ekf.slxc
    ${CMAKE_CURRENT_SOURCE_DIR}/include/riptide_navigation/codegen_compat
  )
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_estimator_support test/test_estimator_support.cpp)
  if(TARGET test_estimator_support)
    target_include_directories(test_estimator_support PRIVATE include)
  endif()
  ament_add_gtest(test_generated_core test/test_generated_core.cpp)
  if(TARGET test_generated_core)
    target_link_libraries(test_generated_core talos_ekf_generated)
  endif()
endif()

ament_package()
