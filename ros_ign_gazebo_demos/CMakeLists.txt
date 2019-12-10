cmake_minimum_required(VERSION 3.5)

project(ros_ign_gazebo_demos)

find_package(ament_cmake REQUIRED)

# TODO: FIX cmake problem with ignition-gazebo3
# CMake Error at /usr/share/dart/cmake/DARTFindurdfdom.cmake:13 (set_target_properties):
#  Property INTERFACE_LINK_LIBRARIES may not contain link-type keyword
#  "optimized".  The INTERFACE_LINK_LIBRARIES property may contain
#  configuration-sensitive generator-expressions which may be used to specify
#  per-configuration rules.
#
# Citadel
#find_package(ignition-gazebo3 QUIET)
#if (NOT ignition-gazebo3_FOUND)
   # Blueprint
#  find_package(ignition-gazebo2 2.1 QUIET REQUIRED)
# endif()

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
