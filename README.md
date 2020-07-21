[![Build Status](https://travis-ci.org/ignitionrobotics/ros_ign.svg?branch=melodic)](https://travis-ci.org/ignitionrobotics/ros_ign/branches)

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

* [ros_ign](https://github.com/osrf/ros_ign/tree/melodic/ros_ign):
  Metapackage which provides all the other packages.
* [ros_ign_image](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_image):
  Unidirectional transport bridge for images from
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  to ROS using
  [image_transport](http://wiki.ros.org/image_transport).
* [ros_ign_bridge](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_bridge):
  Bidirectional transport bridge between
  [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  and ROS.
* [ros_ign_gazebo](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_gazebo):
  Convenient launch files and executables for using
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo)
  with ROS.
* [ros_ign_gazebo_demos](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_gazebo_demos):
  Demos using the ROS-Ignition integration.
* [ros_ign_point_cloud](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_point_cloud):
  Plugins for publishing point clouds to ROS from
  [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo) simulations.

## Install

This branch supports ROS Melodic. See above for other ROS versions.

### Binaries

At the moment, Melodic binaries are only available for Blueprint.
They are hosted at https://packages.osrfoundation.org.

1. Add https://packages.osrfoundation.org

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update

1. Install `ros_ign`

        sudo apt install ros-melodic-ros-ign

### From source

### ROS

Be sure you've installed
[ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (at least ROS-Base).
More ROS dependencies will be installed below.

### Ignition

Install either [Blueprint or Citadel](https://ignitionrobotics.org/docs).

Set the `IGNITION_VERSION` environment variable to the Ignition version you'd
like to compile against. For example:

    export IGNITION_VERSION=citadel

> You only need to set this variable when compiling, not when running.

### Compile ros_ign

The following steps are for Linux and OSX.

1. Create a catkin workspace:

    ```
    # Setup the workspace
    mkdir -p ~/ws/src
    cd ~/ws/src

    # Download needed software
    git clone https://github.com/osrf/ros_ign.git -b melodic
    ```

1. Install ROS dependencies:

    ```
    cd ~/ws
    rosdep install --from-paths src -i -y --rosdistro melodic \
      --skip-keys=ignition-gazebo2 \
      --skip-keys=ignition-gazebo3 \
      --skip-keys=ignition-msgs4 \
      --skip-keys=ignition-msgs5 \
      --skip-keys=ignition-rendering2 \
      --skip-keys=ignition-rendering3 \
      --skip-keys=ignition-sensors2 \
      --skip-keys=ignition-sensors3 \
      --skip-keys=ignition-transport7 \
      --skip-keys=ignition-transport8

    ```

1. Build the workspace:

    ```
    # Source ROS distro's setup.bash
    source /opt/ros/melodic/setup.bash

    # Build and install into workspace
    cd ~/ws/
    catkin_make install
    ```
