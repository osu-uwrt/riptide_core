cmake_minimum_required(VERSION 3.16)
project(riptide_navigation LANGUAGES CXX)

# colcon deploy supplies BUILD_DEPLOY. Optimize deployment builds that have no
# selected configuration, without overriding local builds or explicit choices.
if(BUILD_DEPLOY AND NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build configuration" FORCE)
endif()

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

# Keep install checks tied to the built source/configuration, rather than a
# second hardcoded implementation version or an obsolete tuning frequency.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  src/talos_ekf_node.cpp config/talos_ekf.yaml "${GENERATED_DIR}/talos_ekf.cpp")
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/src/talos_ekf_node.cpp" IMPLEMENTATION_LINE
  REGEX "constexpr char kImplementationVersion")
string(REGEX MATCH "\"([^\"]+)\"" IMPLEMENTATION_MATCH "${IMPLEMENTATION_LINE}")
if(NOT IMPLEMENTATION_MATCH)
  message(FATAL_ERROR "Cannot identify estimator implementation for install verification")
endif()
set(TALOS_IMPLEMENTATION "${CMAKE_MATCH_1}")
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/config/talos_ekf.yaml" TALOS_CONFIG_SHA256)
file(SHA256 "${GENERATED_DIR}/talos_ekf.cpp" TALOS_GENERATED_SHA256)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/estimator_manifest.json"
  "{\n  \"implementation\": \"${TALOS_IMPLEMENTATION}\",\n"
  "  \"architecture\": \"${TALOS_EKF_CODEGEN_ARCH}\",\n"
  "  \"config_sha256\": \"${TALOS_CONFIG_SHA256}\",\n"
  "  \"generated_source_sha256\": \"${TALOS_GENERATED_SHA256}\"\n}\n")

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
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/estimator_manifest.json"
  DESTINATION share/${PROJECT_NAME})
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
set(NAVIGATION_TOOLS
  scripts/analyze_navigation_bag.py
  scripts/record_navigation_run.sh
  scripts/verify_jetson_install.sh)
foreach(NAVIGATION_TOOL IN LISTS NAVIGATION_TOOLS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${NAVIGATION_TOOL}")
    install(PROGRAMS "${NAVIGATION_TOOL}" DESTINATION lib/${PROJECT_NAME})
  endif()
endforeach()
if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/validation")
  install(DIRECTORY validation DESTINATION share/${PROJECT_NAME}
    PATTERN "runs" EXCLUDE)
endif()

# Development tests and replay tools are optional in a runtime-only checkout.
if(BUILD_TESTING AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
    AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/validation")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
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
  add_test(NAME test_paired_statistics
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_paired_statistics.py)
  add_test(NAME test_cohort_statistics
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_cohort_statistics.py)
  add_test(NAME test_truth_reference
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_truth_reference.py)
  add_test(NAME test_covariance_audit
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_covariance_audit.py)
  add_test(NAME test_truth_probe
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_truth_probe.py)
  set_tests_properties(test_truth_probe PROPERTIES
    ENVIRONMENT "MPLCONFIGDIR=${CMAKE_CURRENT_BINARY_DIR}/matplotlib")
  add_test(NAME test_sensor_outage
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_sensor_outage.py)
  add_test(NAME test_sensor_covariance
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_sensor_covariance.py)
  add_test(NAME test_shadow_launch
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_shadow_launch.py)
  set_tests_properties(test_shadow_launch PROPERTIES
    ENVIRONMENT "TALOS_TEST_NODE=$<TARGET_FILE:talos_ekf_node>" TIMEOUT 60)
  add_test(NAME test_record_navigation
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_record_navigation.py)
  add_test(NAME test_benchmark_initial_pose
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_benchmark_initial_pose.py)
  set_tests_properties(test_benchmark_initial_pose PROPERTIES
    ENVIRONMENT "MPLCONFIGDIR=${CMAKE_CURRENT_BINARY_DIR}/matplotlib")
  add_test(NAME test_quaternion_jitter
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/validation -p test_quaternion_jitter.py)
  set_tests_properties(test_quaternion_jitter PROPERTIES
    ENVIRONMENT "MPLCONFIGDIR=${CMAKE_CURRENT_BINARY_DIR}/matplotlib")
  ament_add_gtest(test_estimator_support test/test_estimator_support.cpp)
  if(TARGET test_estimator_support)
    target_include_directories(test_estimator_support PRIVATE include)
  endif()
  ament_add_gtest(test_generated_core test/test_generated_core.cpp)
  if(TARGET test_generated_core)
    target_include_directories(test_generated_core PRIVATE include)
    target_link_libraries(test_generated_core talos_ekf_generated)
  endif()
endif()

ament_package()
