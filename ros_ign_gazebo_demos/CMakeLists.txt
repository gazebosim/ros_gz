cmake_minimum_required(VERSION 3.5)

project(ros_ign_gazebo_demos)

find_package(ament_cmake REQUIRED)

# Citadel
find_package(ignition-gazebo3 QUIET)
if (NOT ignition-gazebo3_FOUND)
  # Blueprint
  find_package(ignition-gazebo2 2.1 QUIET REQUIRED)
endif()

ament_package()

install(
  PROGRAMS
    scripts/ign_gazebo
  RUNTIME DESTINATION bin)

install(
  DIRECTORY
    launch/
  DESTINATION share
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/rviz
)
