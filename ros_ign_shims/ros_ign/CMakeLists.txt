cmake_minimum_required(VERSION 3.5)
project(ros_ign)

find_package(ament_cmake REQUIRED)
message(DEPRECATION "ros_ign is deprecated. Please us ros_gz instead!")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
