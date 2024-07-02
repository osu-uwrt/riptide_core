cmake_minimum_required(VERSION 3.8)
project(riptide_gyro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(riptide_msgs2 REQUIRED)

set(deps
  rclcpp
  riptide_msgs2)

# build and install serial library
file(GLOB serial_library_sources RELATIVE ${PROJECT_SOURCE_DIR}
  src/riptide_gyro/serial_library/*.cpp)

add_library(serial_library ${serial_library_sources})

ament_target_dependencies(serial_library ${deps})

target_include_directories(serial_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  TARGETS serial_library
  EXPORT serial_libraryTargets
  DESTINATION lib/${PROJECT_NAME})

  #export serial library
ament_export_targets(serial_libraryTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps})

# build and install driver
add_executable(gyro_driver
  src/riptide_gyro/driver.cpp)

ament_target_dependencies(gyro_driver ${deps})

target_link_libraries(gyro_driver
  serial_library)

target_include_directories(gyro_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS gyro_driver
  DESTINATION lib/${PROJECT_NAME})

#install config and launch
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

#install include directory for lib
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET) 

  if(ament_cmake_gtest_FOUND)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)

    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    # glob test sources together
    file(GLOB test_serial_library_sources RELATIVE ${PROJECT_SOURCE_DIR} 
      test/riptide_gyro/serial_library/*.cpp)

    #
    # REGULAR SERIAL LIBRARY TEST
    #
    ament_add_gtest(test_serial_library
      ${test_serial_library_sources}
      test/riptide_gyro/RosTest.cpp
      test/riptide_gyro/LinuxTransceiverTest.cpp
      test/riptide_gyro/SerialProcessorTests.cpp
      test/riptide_gyro/ExecuteTests.cpp)
    
    target_include_directories(test_serial_library PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(test_serial_library
      ${deps})
    
    target_compile_options(test_serial_library PUBLIC -Wno-unused-parameter)
    
    target_compile_definitions(test_serial_library PUBLIC TESTING_ENABLED)
    
    target_link_libraries(test_serial_library serial_library)

    install(TARGETS test_serial_library
      DESTINATION lib/${PROJECT_NAME})

    # #
    # # ARDUINO SERIAL LIBRARY TEST
    # #
    # ament_add_gtest(test_serial_library_arduino
    #   ${test_serial_library_sources}
    #   test/riptide_gyro/RosTest.cpp
    #   test/riptide_gyro/LinuxTransceiverTest.cpp
    #   test/riptide_gyro/SerialProcessorTests.cpp
    #   test/riptide_gyro/ExecuteTests.cpp)
    
    # target_compile_definitions(test_serial_library_arduino PUBLIC FORCE_ARDUINO)
    
    # target_include_directories(test_serial_library_arduino PUBLIC
    #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #   $<INSTALL_INTERFACE:include>)

    # ament_target_dependencies(test_serial_library_arduino
    #   ${deps})
    
    # target_link_libraries(test_serial_library_arduino serial_library)
    
    # install(TARGETS test_serial_library_arduino
    #   DESTINATION lib/${PROJECT_NAME})
    
    ament_add_gtest(test_arduino_lib
      test/riptide_gyro/TestArduinoLib.cpp)
    
    target_compile_definitions(test_arduino_lib PUBLIC FORCE_ARDUINO TESTING_ENABLED)

    # dont really care about pedantic iso c++ type warnings for now
    target_compile_options(test_arduino_lib PUBLIC -Wno-pedantic)

    target_include_directories(test_arduino_lib PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(test_arduino_lib
      ${deps})

    install(TARGETS test_arduino_lib
      DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

ament_package()
