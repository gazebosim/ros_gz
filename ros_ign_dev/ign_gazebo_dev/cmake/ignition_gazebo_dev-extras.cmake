# This code here calls and sets required igniton gazebo based on ROS Distribution so,packages can directly go find_package(ign_gazebo_dev) so,that 
# they don't need to change code with different ros distros
set( ENV{OVERRIDE_ROS_GZ_VERSION} 6 )

if(DEFINED ENV{ROS_DISTRO})
message(STATUS "Detected ROS Distro is $ENV{ROS_DISTRO}")
endif()

if("$ENV{ROS_DISTRO}" MATCHES "foxy")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
elseif("$ENV{ROS_DISTRO}" MATCHES "galactic")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
elseif("$ENV{ROS_DISTRO}" MATCHES "humble")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
else()
  message(STATUS "No ROS Distro detected")
  message(STATUS "Setting to default gazebo version")
  find_package(ignition-gazebo$ENV{OVERRIDE_ROS_GZ_VERSION} REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo$ENV{OVERRIDE_ROS_GZ_VERSION}_VERSION_MAJOR})

endif()
