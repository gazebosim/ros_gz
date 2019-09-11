# Bridge communication between ROS and Ignition Transport

This package provides a network bridge which enables the exchange of messages
between ROS and Ignition Transport.

The bridge is currently implemented in C++. At this point there's no support for
service calls. Its support is limited to only the following message types:

| ROS type                     | Ignition Transport type          |
|--------------------------------|:--------------------------------:|
| TODO                           | TODO                             |

## Prerequisites

* ROS 2 [Dashing](https://index.ros.org/doc/ros2/Installation/Dashing)
* Ignition [Blueprint](https://ignitionrobotics.org/docs/blueprint/install)

### Building the bridge from source

Before continuing you should have the prerequisites for building the bridge from
source installed.

1. Create a colcon workspace:

```
# Setup the workspace
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src

# Download needed software
git clone https://github.com/osrf/ros_ign.git
```

2. Build the workspace:

```
# Source ROS distro's setup.bash
source /opt/ros/dashing/setup.bash

# Build and install into workspace
cd ~/bridge_ws/
colcon build
```

## Example 1a: Ignition Transport talker and ROS 2 listener

TODO

## Example 1b: ROS 2 talker and Ignition Transport listener

TODO

## Example 2: Run the bridge and exchange images

TODO

## Example 3: Static bridge

TODO
