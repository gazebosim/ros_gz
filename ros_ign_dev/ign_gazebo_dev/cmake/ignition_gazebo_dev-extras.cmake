# This code here calls and sets required igniton gazebo based on ROS Distribution so,packages can directly go find_package(ign_gazebo_dev) so,that 
# they don't need to change code with different ros distros
if(DEFINED ENV{ROS_DISTRO})    
message(STATUS "Detected ROS Distro is $ENV{ROS_DISTRO}")

if("$ENV{ROS_DISTRO}" MATCHES "foxy")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
elseif("$ENV{ROS_DISTRO}" MATCHES "galactic")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
elseif("$ENV{ROS_DISTRO}" MATCHES "humble")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
else()
message(STATUS "No ROS Distro detected")
endif()
