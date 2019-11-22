# Bridge communication between ROS and Ignition Transport

This package provides a network bridge which enables the exchange of messages
between ROS and Ignition Transport.

The bridge is currently implemented in C++. At this point there's no support for
service calls. Its support is limited to only the following message types:

| ROS type                       | Ignition Transport type          |
|--------------------------------|:--------------------------------:|
| std_msgs/Bool                  | ignition::msgs::Boolean          |
| std_msgs/Empty                 | ignition::msgs::Empty            |
| std_msgs/Float32               | ignition::msgs::Float            |
| std_msgs/Header                | ignition::msgs::Header           |
| std_msgs/String                | ignition::msgs::StringMsg        |
| geometry_msgs/Quaternion       | ignition::msgs::Quaternion       |
| geometry_msgs/Vector3          | ignition::msgs::Vector3d         |
| geometry_msgs/Point            | ignition::msgs::Vector3d         |
| geometry_msgs/Pose             | ignition::msgs::Pose             |
| geometry_msgs/PoseStamped      | ignition::msgs::Pose             |
| geometry_msgs/Transform        | ignition::msgs::Pose             |
| geometry_msgs/TransformStamped | ignition::msgs::Pose             |
| geometry_msgs/Twist            | ignition::msgs::Twist            |
| mav_msgs/Actuators             | ignition::msgs::Actuators        |
| nav_msgs/Odometry              | ignition::msgs::Odometry         |
| rosgraph_msgs/Clock            | ignition::msgs::Clock            |
| sensor_msgs/BatteryState       | ignition::msgs::BatteryState     |
| sensor_msgs/CameraInfo         | ignition::msgs::CameraInfo       |
| sensor_msgs/Imu                | ignition::msgs::IMU              |
| sensor_msgs/Image              | ignition::msgs::Image            |
| sensor_msgs/JointState         | ignition::msgs::Model            |
| sensor_msgs/LaserScan          | ignition::msgs::LaserScan        |
| sensor_msgs/MagneticField      | ignition::msgs::Magnetometer     |
| sensor_msgs/PointCloud2        | ignition::msgs::PointCloudPacked |

Run `parameter_bridge -h` for instructions.

## Prerequisites

For all examples you need to source the environment of the install space where
the bridge was built or unpacked to.
Additionally you will need to either source the ROS environment or at least
set the `ROS_MASTER_URI` and run a `roscore`.

The following ROS packages are required to build and use the bridge:
* `catkin`
* `geometry_msgs`
* `mav_msgs`
* `nav_msgs`
* `rosconsole`
* `roscpp`
* `roslaunch` (for `roscore` executable)
* `rosmsg`
* `sensor_msgs`
* `std_msgs`

To run the following examples you will also need these ROS packages:
* `rosbash` (for `rosrun` executable)
* `roscpp_tutorials`
* `rostopic`
* `rqt_image_view`

The following Ignition dependencies are also needed:
* `libignition-msgs4-dev`
* `libignition-transport7-dev`

### Building the bridge from source

Before continuing you should have the prerequisites for building the bridge from
source installed.

Here are the steps (for Linux and OSX; you probably don't have ROS installed
on Windows).

1. Create a catkin workspace:

```
# Setup the workspace
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src

# Download needed software
git clone https://github.com/osrf/ros_ign_bridge.git
```

2. Build the workspace:

```
# Source ROS distro's setup.bash
source /opt/ros/melodic/setup.bash

# Build and install into workspace
cd ~/bridge_ws/
catkin_make install
```

## Example 1a: Ignition Transport talker and ROS listener

First we start a ROS `roscore`:

```
# Shell A:
. /opt/ros/melodic/setup.bash
roscore
```

Then we start the parameter bridge which will watch the specified topics.

```
# Shell B:
. ~/bridge_ws/install/setup.bash
rosrun ros_ign_bridge parameter_bridge /chatter@std_msgs/String@ignition.msgs.StringMsg
```

Now we start the ROS listener.

```
# Shell C:
. /opt/ros/melodic/setup.bash
rostopic echo /chatter
```

Now we start the Ignition Transport talker.

```
# Shell D:
ign topic pub -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'
```

## Example 1b: ROS talker and Ignition Transport listener

First we start a ROS `roscore`:

```
# Shell A:
. /opt/ros/melodic/setup.bash
roscore
```

Then we start the parameter bridge which will watch the specified topics.

```
# Shell B:
. ~/bridge_ws/install/setup.bash
rosrun ros_ign_bridge parameter_bridge /chatter@std_msgs/String@ignition.msgs.StringMsg
```

Now we start the Ignition Transport listener.

```
# Shell C:
ign topic -e -t /chatter
```

Now we start the ROS talker.

```
# Shell D:
. /opt/ros/melodic/setup.bash
rostopic pub /chatter std_msgs/String "data: 'Hi'" --once
```

## Example 2: Run the bridge and exchange images

In this example, we're going to generate Ignition Transport images using Gazebo,
that will be converted into ROS images, and visualized with `rqt_viewer`.

First we start a ROS `roscore`:

```
# Shell A:
. /opt/ros/melodic/setup.bash
roscore
```

Then we start Gazebo.

```
# Shell B:
gazebo
```

Once Gazebo is running, click on the `Insert` tab, and then, insert a `Camera`
object into the scene. Now, let's see the topic where the camera images are
published.

```
# Shell C:
ign topic -l | grep image
/default/camera/link/camera/image
```

Then we start the parameter bridge with the previous topic.

```
# Shell D:
. ~/bridge_ws/install/setup.bash
rosrun ros_ign_bridge parameter_bridge /default/camera/link/camera/image@sensor_msgs/Image@ignition.msgs.Image
```

Now we start the ROS GUI:

```
# Shell E:
. /opt/ros/melodic/setup.bash
rqt_image_view /default/camera/link/camera/image
```

You should see the current images in `rqt_image_view` which are coming from
Gazebo (published as Ignition Msgs over Ignition Transport).

The screenshot shows all the shell windows and their expected content
(it was taken using ROS Kinetic):

![Ignition Transport images and ROS rqt](images/bridge_image_exchange.png)
