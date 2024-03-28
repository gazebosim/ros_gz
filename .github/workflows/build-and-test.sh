#!/bin/bash
set -ev

# Configuration.
export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

apt update -qq
apt install -qq -y lsb-release wget curl build-essential git

if [ "$GZ_VERSION" == "garden" ]; then
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
  wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

  GZ_DEPS="libgz-sim7-dev"

  ROSDEP_ARGS="--skip-keys='sdformat-urdf'"
elif [ "$GZ_VERSION" == "harmonic" ]; then
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list
  wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

  GZ_DEPS="libgz-sim8-dev"

  ROSDEP_ARGS="--skip-keys='sdformat-urdf'"
fi

# Fortress comes through rosdep for Focal and Jammy

# Dependencies.
echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-testing.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y $GZ_DEPS \
                       python3-colcon-common-extensions \
                       python3-rosdep

rosdep init
# Temporarily pin rosdep to fix CI on rolling
sed -i "s|ros\/rosdistro\/master|ros\/rosdistro\/rolling\/2024-03-13|" /etc/ros/rosdep/sources.list.d/20-default.list
export ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/rolling/2024-03-13/index-v4.yaml
# end Temporary fix

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
