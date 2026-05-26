#!/bin/bash
set -ev

# Configuration.
export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

apt update -qq
apt install -qq -y lsb-release wget curl gnupg build-essential ca-certificates

# Use signed-by keyring files. `apt-key add` is removed on Ubuntu 24.04
# (noble), so the previous form silently produced untrusted apt sources
# and `ros-rolling-*` was never installable.
install -d -m 0755 /etc/apt/keyrings

curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /etc/apt/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2-testing.list

apt-get update -qq
# Install build dependencies explicitly. On this image `rosdep update` only
# fetches the base/python/ruby YAMLs and does not pull the rolling
# distribution.yaml, so ROS-released packages like gz_*_vendor, actuator_msgs,
# gps_msgs, vision_msgs, etc. silently fail to install.
#
# ros-$ROS_DISTRO-desktop covers ros-base, common_interfaces (geometry_msgs,
# nav_msgs, sensor_msgs, …), image_transport, rviz2, rqt-*,
# robot_state_publisher, launch_testing_ament_cmake, ament_lint_common.
apt-get install -y python3-colcon-common-extensions \
                   python3-rosdep \
                   libcli11-dev \
                   ros-$ROS_DISTRO-desktop \
                   ros-$ROS_DISTRO-gz-math-vendor \
                   ros-$ROS_DISTRO-gz-msgs-vendor \
                   ros-$ROS_DISTRO-gz-sim-vendor \
                   ros-$ROS_DISTRO-gz-transport-vendor \
                   ros-$ROS_DISTRO-yaml-cpp-vendor \
                   ros-$ROS_DISTRO-actuator-msgs \
                   ros-$ROS_DISTRO-gps-msgs \
                   ros-$ROS_DISTRO-marine-acoustic-msgs \
                   ros-$ROS_DISTRO-vision-msgs \
                   ros-$ROS_DISTRO-trajectory-msgs \
                   ros-$ROS_DISTRO-simulation-interfaces \
                   ros-$ROS_DISTRO-image-transport-plugins \
                   ros-$ROS_DISTRO-rviz-imu-plugin \
                   ros-$ROS_DISTRO-sdformat-urdf \
                   ros-$ROS_DISTRO-xacro

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO $ROSDEP_ARGS

# Build.
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
cd $COLCON_WS
colcon build --event-handlers console_direct+

# Tests.
colcon test --event-handlers console_direct+
colcon test-result
