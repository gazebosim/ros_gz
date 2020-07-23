[![Build Status](https://travis-ci.org/ignitionrobotics/ros_ign.svg?branch=noetic)](https://travis-ci.org/ignitionrobotics/ros_ign/branches)

* ROS 1 branches:
    * [melodic](https://github.com/osrf/ros_ign/tree/melodic)
        * Blueprint and Citadel
        * Melodic
    * [noetic](https://github.com/osrf/ros_ign/tree/noetic)
        * Citadel
        * Noetic
* ROS 2 branches:
    * [dashing](https://github.com/osrf/ros_ign/tree/dashing)
        * Blueprint and Citadel
        * Dashing and Eloquent
    * [ros2](https://github.com/osrf/ros_ign/tree/ros2)
        * Citadel
        * Foxy

# Integration between ROS and Ignition

## Packages

This repository holds packages that provide integration between
[ROS](http://www.ros.org/) and [Ignition](https://ignitionrobotics.org):

* [ros_ign](https://github.com/osrf/ros_ign/tree/noetic/ros_ign):
  Metapackage which provides all the other packages.
* [ros_ign_image](https://github.com/osrf/ros_ign/tree/noetic/ros_ign_image):
  Unidirectional transport bridge for images from
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  to ROS using
  [image_transport](http://wiki.ros.org/image_transport).
* [ros_ign_bridge](https://github.com/osrf/ros_ign/tree/noetic/ros_ign_bridge):
  Bidirectional transport bridge between
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  and ROS.
* [ros_ign_gazebo](https://github.com/osrf/ros_ign/tree/noetic/ros_ign_gazebo):
  Convenient launch files and executables for using
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo)
  with ROS.
* [ros_ign_gazebo_demos](https://github.com/osrf/ros_ign/tree/noetic/ros_ign_gazebo_demos):
  Demos using the ROS-Ignition integration.
* [ros_ign_point_cloud](https://github.com/osrf/ros_ign/tree/noetic/ros_ign_point_cloud):
  Plugins for publishing point clouds to ROS from
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo) simulations.

## Install

This branch supports ROS Noetic. See above for other ROS versions.

### ROS

Be sure you've installed
[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (at least ROS-Base).

### Binaries

Noetic binaries will *soon* be available for Citadel.
They will be hosted at https://packages.ros.org.

1. Make sure you have ROS Noetic installed.

1. Install `ros_ign`

        sudo apt install ros-noetic-ros-ign

### From source

The following steps are for Linux and OSX.

1. Create a catkin workspace:

    ```
    # Setup the workspace
    mkdir -p ~/ws/src
    cd ~/ws/src

    # Download needed software
    git clone https://github.com/osrf/ros_ign.git -b noetic
    ```

1. Install dependencies (this will also install Ignition):

    ```
    cd ~/ws
    rosdep install --from-paths src -i -y --rosdistro noetic
    ```

1. Build the workspace:

    ```
    # Source ROS distro's setup.bash
    source /opt/ros/noetic/setup.bash

    # Build and install into workspace
    cd ~/ws/
    catkin_make install
    ```
