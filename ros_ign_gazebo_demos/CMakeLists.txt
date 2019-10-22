cmake_minimum_required(VERSION 3.5)

project(ros_ign_gazebo_demos)

find_package(catkin REQUIRED)

# Citadel
find_package(ignition-gazebo3 QUIET)
if (NOT ignition-gazebo3_FOUND)
  # Blueprint
  find_package(ignition-gazebo2 QUIET)
  if (NOT ignition-gazebo2_FOUND)
    BUILD_ERROR ("Missing: Ignition Gazebo version 2 or 3")
  endif()
endif()

catkin_package()

install(
  PROGRAMS
    scripts/ign_gazebo
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY
    rviz/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

