cmake_minimum_required(VERSION 3.5)

project(ros_ign_gazebo)

find_package(catkin REQUIRED)

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

