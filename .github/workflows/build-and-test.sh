#!/bin/bash
set -ev

# Configuration.
export CATKIN_WS=~/catkin_ws
export CATKIN_WS_SRC=${CATKIN_WS}/src
export DEBIAN_FRONTEND=noninteractive

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

if [ "$IGNITION_VERSION" == "blueprint" ]; then
  IGN_DEPS="libignition-gazebo2-dev"
elif [ "$IGNITION_VERSION" == "citadel" ]; then
  IGN_DEPS="libignition-gazebo3-dev"
elif [ "$IGNITION_VERSION" == "dome" ]; then
  IGN_DEPS="libignition-gazebo4-dev"
elif [ "$IGNITION_VERSION" == "fortress" ]; then
  IGN_DEPS="libignition-gazebo6-dev"
else
  exit 1
fi

# Dependencies.
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
apt-get update -qq
apt-get install -qq -y $IGN_DEPS \
                       python-catkin-tools \
                       python-rosdep

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO

# Build.
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p $CATKIN_WS_SRC
ln -s $GITHUB_WORKSPACE $CATKIN_WS_SRC
cd $CATKIN_WS
catkin init
catkin config --install
catkin build --limit-status-rate 0.1 --no-notify -DCMAKE_BUILD_TYPE=Release
catkin build --limit-status-rate 0.1 --no-notify --make-args tests

# Tests.
catkin run_tests
