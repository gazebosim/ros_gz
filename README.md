[![Build Status](https://github.com/ignitionrobotics/ros_ign/actions/workflows/noetic-ci.yml/badge.svg?branch=noetic)](https://github.com/ignitionrobotics/ros_ign/actions/workflows/noetic-ci.yml)

ROS version | Ignition version | Branch | Binaries hosted at
-- | -- | -- | --
Melodic | Citadel | [melodic](https://github.com/osrf/ros_ign/tree/melodic) | only from source
Melodic | Fortress | [melodic](https://github.com/osrf/ros_ign/tree/melodic) | only from source
Noetic | Citadel | [noetic](https://github.com/osrf/ros_ign/tree/noetic) | https://packages.ros.org
Noetic | Edifice | [noetic](https://github.com/osrf/ros_ign/tree/noetic) | only from source
Noetic | Fortress (not released) | [noetic](https://github.com/osrf/ros_ign/tree/noetic) | only from source
Foxy | Citadel | [foxy](https://github.com/osrf/ros_ign/tree/foxy) | https://packages.ros.org
Foxy | Edifice | [foxy](https://github.com/osrf/ros_ign/tree/foxy) | only from source
Galactic | Edifice | [ros2](https://github.com/osrf/ros_ign/tree/ros2) | https://packages.ros.org
Rolling | Edifice | [ros2](https://github.com/osrf/ros_ign/tree/ros2) | https://packages.ros.org
Rolling | Fortress (not released) | [ros2](https://github.com/osrf/ros_ign/tree/ros2) | only from source

> Please [ticket an issue](https://github.com/ignitionrobotics/ros_ign/issues/) if you'd like support to be added for some combination.

# Integration between ROS and Ignition

## Packages

This repository holds packages that provide integration between
[ROS](http://www.ros.org/) and [Ignition](https://ignitionrobotics.org):

* [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/noetic/ros_ign):
  Metapackage which provides all the other packages.
* [ros_ign_image](https://github.com/ignitionrobotics/ros_ign/tree/noetic/ros_ign_image):
  Unidirectional transport bridge for images from
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  to ROS using
  [image_transport](http://wiki.ros.org/image_transport).
* [ros_ign_bridge](https://github.com/ignitionrobotics/ros_ign/tree/noetic/ros_ign_bridge):
  Bidirectional transport bridge between
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  and ROS.
* [ros_ign_gazebo](https://github.com/ignitionrobotics/ros_ign/tree/noetic/ros_ign_gazebo):
  Convenient launch files and executables for using
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo)
  with ROS.
* [ros_ign_gazebo_demos](https://github.com/ignitionrobotics/ros_ign/tree/noetic/ros_ign_gazebo_demos):
  Demos using the ROS-Ignition integration.
* [ros_ign_point_cloud](https://github.com/ignitionrobotics/ros_ign/tree/noetic/ros_ign_point_cloud):
  Plugins for publishing point clouds to ROS from
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo) simulations.

## Install

This branch supports ROS Noetic. See above for other ROS versions.

### Binaries

Noetic binaries are only available for Citadel.
They are hosted at https://packages.ros.org.

1. Add https://packages.ros.org

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        sudo apt-get update

1. Install `ros_ign`

        sudo apt install ros-noetic-ros-ign

### From source

#### ROS

Be sure you've installed
[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (at least ROS-Base).
More ROS dependencies will be installed below.

#### Ignition

Install either [Citadel, Edifice or Fortress](https://ignitionrobotics.org/docs).

Set the `IGNITION_VERSION` environment variable to the Ignition version you'd
like to compile against. For example:

    export IGNITION_VERSION=citadel

> You only need to set this variable when compiling, not when running.

#### Compile ros_ign

The following steps are for Linux and OSX.

1. Create a catkin workspace:

    ```
    # Setup the workspace
    mkdir -p ~/ws/src
    cd ~/ws/src

    # Download needed software
    git clone https://github.com/osrf/ros_ign.git -b noetic
    ```

1. Install dependencies (this may also install Ignition):

    ```
    cd ~/ws
    rosdep install -r --from-paths src -i -y --rosdistro noetic
    ```

    > If `rosdep` fails to install Ignition libraries and you have not installed them before, please follow [Ignition installation instructions](https://ignitionrobotics.org/docs/latest/install).

1. Build the workspace:

    ```
    # Source ROS distro's setup.bash
    source /opt/ros/noetic/setup.bash

    # Build and install into workspace
    cd ~/ws/
    catkin_make install
    ```
