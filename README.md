[![Build Status](https://travis-ci.org/ignitionrobotics/ros_ign.svg?branch=ros2)](https://travis-ci.org/ignitionrobotics/ros_ign/branches)

* ROS 1 branches:
    * [melodic](https://github.com/ignitionrobotics/ros_ign/tree/melodic)
        * Blueprint and Citadel
        * Melodic
* ROS 2 branches:
    * [dashing](https://github.com/ignitionrobotics/ros_ign/tree/dashing)
        * Blueprint and Citadel
        * Dashing and Eloquent
    * [ros2](https://github.com/osrf/ros_ign/tree/ros2)
        * Citadel
        * Foxy

# Integration between ROS and Ignition

This repository holds packages that provide integration between
[ROS](http://www.ros.org/) and [Ignition](https://ignitionrobotics.org):

* [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/dashing/ros_ign):
  Metapackage which provides all the other packages.
* [ros_ign_image](https://github.com/ignitionrobotics/ros_ign/tree/dashing/ros_ign_image):
  Unidirectional transport bridge for images from
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  to ROS using
  [image_transport](http://wiki.ros.org/image_transport).
* [ros_ign_bridge](https://github.com/ignitionrobotics/ros_ign/tree/dashing/ros_ign_bridge):
  Bidirectional transport bridge between
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  and ROS.
* [ros_ign_gazebo](https://github.com/ignitionrobotics/ros_ign/tree/dashing/ros_ign_gazebo):
  Convenient launch files and executables for using
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo)
  with ROS.
* [ros_ign_gazebo_demos](https://github.com/ignitionrobotics/ros_ign/tree/dashing/ros_ign_gazebo_demos):
  Demos using the ROS-Ignition integration.
* [ros_ign_point_cloud](https://github.com/ignitionrobotics/ros_ign/tree/dashing/ros_ign_point_cloud):
  Plugins for publishing point clouds to ROS from
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo) simulations.
