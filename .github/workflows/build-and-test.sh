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
# Bootstrap: tools rosdep itself needs, plus `ros-base` so /opt/ros/$ROS_DISTRO
# exists to source. Every workspace dependency is then resolved by rosdep
# against the rolling distribution.yaml.
apt-get install -y python3-colcon-common-extensions \
                   python3-rosdep \
                   libcli11-dev \
                   ros-$ROS_DISTRO-ros-base

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths ./ -i -y --rosdistro $ROS_DISTRO $ROSDEP_ARGS

# Build.
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
cd $COLCON_WS
colcon build --event-handlers console_direct+

# Tests.
colcon test --event-handlers console_direct+
colcon test-result
