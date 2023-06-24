cmake_minimum_required(VERSION 3.5)
find_package(ament_cmake REQUIRED)
project(ign_gazebo_dev)

ament_package(
CONFIG_EXTRAS cmake/ignition_gazebo_dev-extras.cmake)
