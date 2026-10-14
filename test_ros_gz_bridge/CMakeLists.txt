cmake_minimum_required(VERSION 3.5)

project(test_ros_gz_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(ros_gz_bridge REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  ament_add_gtest(test_ros_gz_bridge src/test_ros_gz_bridge.cpp)
  target_link_libraries(test_ros_gz_bridge ament_cmake_ros_core::ament_ros_defaults ros_gz_bridge::ros_gz_bridge)
endif()

ament_package()
