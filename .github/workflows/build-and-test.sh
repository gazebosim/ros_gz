#!/bin/bash
set -ev

# Configuration.
export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

# Citadel gets Ignition with rosdep
if [ "$IGNITION_VERSION" != "citadel" ]; then
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list
  wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

  if [ "$IGNITION_VERSION" == "dome" ]; then
    IGN_DEPS="libignition-msgs6-dev libignition-transport9-dev libignition-gazebo4-dev"
  fi

  if [ "$IGNITION_VERSION" == "edifice" ]; then
    IGN_DEPS="libignition-msgs7-dev libignition-transport10-dev libignition-gazebo5-dev"
  fi

  if [ "$IGNITION_VERSION" == "fortress" ]; then
    IGN_DEPS="libignition-msgs8-dev libignition-transport11-dev libignition-gazebo6-dev"
  fi
fi

# Dependencies.
echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y $IGN_DEPS \
                       python3-colcon-common-extensions \
                       python3-rosdep

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO

# Build.
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
cd $COLCON_WS
colcon build --event-handlers console_direct+

# Tests.
colcon test --event-handlers console_direct+
colcon test-result
