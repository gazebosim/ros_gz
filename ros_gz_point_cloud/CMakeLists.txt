cmake_minimum_required(VERSION 3.20)

project(ros_gz_point_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
# Provides the ament_cmake_ros_core::ament_ros_cxx_standard interface target,
# which sets the C++ standard (C++20 on Rolling) used across the ROS 2 core.
find_package(ament_cmake_ros_core REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

set(plugin_name RosGzPointCloud)
add_library(${plugin_name} SHARED
  src/point_cloud.cpp
)
target_include_directories(${plugin_name} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${plugin_name}
  ament_cmake_ros_core::ament_ros_cxx_standard
  gz-common::gz-common
  gz-math::gz-math
  gz-plugin::register
  gz-rendering::gz-rendering
  gz-sim::gz-sim
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    examples/
  DESTINATION
    share/${PROJECT_NAME}/examples
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
