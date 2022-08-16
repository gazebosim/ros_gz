cmake_minimum_required(VERSION 3.5)

project(ros_ign_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
endif()

ign_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)

add_executable(create src/create.cpp)
ament_target_dependencies(create
  rclcpp
  ignition-math6
  std_msgs
)
target_link_libraries(create
  gflags
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

add_library(Stopwatch src/Stopwatch.cpp)
ament_target_dependencies(Stopwatch
  rclcpp
)
target_include_directories(Stopwatch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

configure_file(
  launch/ign_gazebo.launch.py.in
  launch/ign_gazebo.launch.py.configured
  @ONLY
)
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/ign_gazebo.launch.py"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/ign_gazebo.launch.py.configured"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/launch/ign_gazebo.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  create
  Stopwatch
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

   # GTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  # launch_testing
  find_package(launch_testing_ament_cmake REQUIRED)

  # Helper function to generate gtest
  function(gz_add_gtest TEST_NAME)
    set(
      options
        ROS
        LAUNCH_TEST
    )
    set(oneValueArgs)
    set(multiValueArgs
      EXTRA_ROS_PKGS)

    cmake_parse_arguments(gz_add_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    ament_add_gtest_executable(${TEST_NAME}
      test/${TEST_NAME}.cpp
    )
    target_link_libraries(${TEST_NAME}
      ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
    )
    if(gz_add_test_ROS)
      set(ROS_PKGS rclcpp ${gz_add_test_EXTRA_ROS_PKGS})
      foreach(PKG ${ROS_PKGS})
        find_package(${PKG} REQUIRED)
      endforeach()
      ament_target_dependencies(${TEST_NAME} ${ROS_PKGS})
    endif()
    target_include_directories(${TEST_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(${TEST_NAME}
      Stopwatch
    )
    install(
      TARGETS ${TEST_NAME}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(gz_add_test_LAUNCH_TEST)
      add_launch_test(test/launch/${TEST_NAME}.launch.py
        TIMEOUT 300
      )
    else()
      ament_add_gtest_test(${TEST_NAME})
    endif()
  endfunction()

  # Add gtests
  gz_add_gtest(test_stopwatch ROS)

endif()

ament_package()
