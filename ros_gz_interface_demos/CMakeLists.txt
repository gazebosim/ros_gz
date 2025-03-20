cmake_minimum_required(VERSION 3.20)
project(ros_gz_interface_demos)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(CLI11 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

include_directories(include)

# Spawn Entity executable
add_executable(spawn_entity src/spawn_entity.cpp)
ament_target_dependencies(spawn_entity
  builtin_interfaces
  geometry_msgs
  rclcpp
  ros_gz_interfaces
)

# Delete Entity executable
add_executable(delete_entity src/delete_entity.cpp)
ament_target_dependencies(delete_entity
  builtin_interfaces
  geometry_msgs
  rclcpp
  ros_gz_interfaces
)

# Set Entity Pose executable
add_executable(set_entity_pose src/set_entity_pose.cpp)
ament_target_dependencies(set_entity_pose
  builtin_interfaces
  geometry_msgs
  rclcpp
  ros_gz_interfaces
)

# Install executables
install(TARGETS
  delete_entity
  set_entity_pose
  spawn_entity
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()