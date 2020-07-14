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
| std_msgs/Int32                 | ignition::msgs::Int32            |
| std_msgs/String                | ignition::msgs::StringMsg        |
| geometry_msgs/Quaternion       | ignition::msgs::Quaternion       |
| geometry_msgs/Vector3          | ignition::msgs::Vector3d         |
| geometry_msgs/Point            | ignition::msgs::Vector3d         |
| geometry_msgs/Pose             | ignition::msgs::Pose             |
| geometry_msgs/PoseStamped      | ignition::msgs::Pose             |
| geometry_msgs/Transform        | ignition::msgs::Pose             |
| geometry_msgs/TransformStamped | ignition::msgs::Pose             |
| geometry_msgs/Twist            | ignition::msgs::Twist            |
| mav_msgs/Actuators (TODO)      | ignition::msgs::Actuators (TODO) |
| nav_msgs/Odometry              | ignition::msgs::Odometry         |
| rosgraph_msgs/Clock            | ignition::msgs::Clock            |
| sensor_msgs/BatteryState       | ignition::msgs::BatteryState     |
| sensor_msgs/CameraInfo         | ignition::msgs::CameraInfo       |
| sensor_msgs/FluidPressure (TODO) | ignition::msgs::FluidPressure (TODO) |
| sensor_msgs/Imu                | ignition::msgs::IMU              |
| sensor_msgs/Image              | ignition::msgs::Image            |
| sensor_msgs/JointState         | ignition::msgs::Model            |
| sensor_msgs/LaserScan          | ignition::msgs::LaserScan        |
| sensor_msgs/MagneticField      | ignition::msgs::Magnetometer     |
| sensor_msgs/PointCloud2        | ignition::msgs::PointCloudPacked |

Run `ros2 run ros_ign_bridge parameter_bridge -h` for instructions.

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
    git clone https://github.com/osrf/ros_ign.git -b dashing
    ```

1. Install ROS dependencies:

    ```
    cd ~/bridge_ws
    rosdep install --from-paths src -i -y --rosdistro dashing \
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
    source /opt/ros/dashing/setup.bash

    # Build and install into workspace
    cd ~/bridge_ws/
    colcon build
    ```

## Example 1a: Ignition Transport talker and ROS 2 listener

Start the parameter bridge which will watch the specified topics.

```
# Shell A:
. ~/bridge_ws/install/setup.bash
ros2 run ros_ign_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
```

Now we start the ROS listener.

```
# Shell B:
. /opt/ros/melodic/setup.bash
ros2 topic echo /chatter
```

Now we start the Ignition Transport talker.

```
# Shell C:
ign topic pub -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'
```

## Example 1b: ROS 2 talker and Ignition Transport listener

Start the parameter bridge which will watch the specified topics.

```
# Shell A:
. ~/bridge_ws/install/setup.bash
ros2 run ros_ign_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
```

Now we start the Ignition Transport listener.

```
# Shell B:
ign topic -e -t /chatter
```

Now we start the ROS talker.

```
# Shell C:
. /opt/ros/melodic/setup.bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hi'" --once
```

## Example 2: Run the bridge and exchange images

In this example, we're going to generate Ignition Transport images using
Ignition Gazebo, that will be converted into ROS images, and visualized with
`rqt_image_viewer`.

First we start Ignition Gazebo.

```
# Shell A:
ign gazebo sensors_demo.sdf
```

Let's see the topic where camera images are published.

```
# Shell B:
ign topic -l | grep image
/rgbd_camera/image
```

Then we start the parameter bridge with the previous topic.

```
# Shell B:
. ~/bridge_ws/install/setup.bash
ros2 run ros_ign_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image
```

Now we start the ROS GUI:

```
# Shell C:
. /opt/ros/melodic/setup.bash
ros2 run rqt_image_view rqt_image_view /rgbd_camera/image
```

You should see the current images in `rqt_image_view` which are coming from
Gazebo (published as Ignition Msgs over Ignition Transport).

The screenshot shows all the shell windows and their expected content
(it was taken using ROS Kinetic):

![Ignition Transport images and ROS rqt](images/bridge_image_exchange.png)

## Example 3: Static bridge

In this example, we're going to run an executable that starts a bidirectional
bridge for a specific topic and message type. We'll use the `static_bridge`
executable that is installed with the bridge.

The example's code can be found under `ros_ign_bridge/src/static_bridge.cpp`.
In the code, it's possible to see how the bridge is hardcoded to bridge string
messages published on the `/chatter` topic.

Let's give it a try, starting with Ignition -> ROS 2.

On terminal A, start the bridge:

`ros2 run ros_ign_bridge static_bridge`

On terminal B, we start a ROS 2 listener:

`ros2 topic echo /chatter std_msgs/msg/String`

And terminal C, publish an Ignition message:

`ign topic pub -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'`

At this point, you should see the ROS 2 listener echoing the message.

Now let's try the other way around, ROS 2 -> Ignition.

On terminal D, start an Igntion listener:

`ign topic -e -t /chatter`

And on terminal E, publish a ROS 2 message:

`ros2 topic pub /chatter std_msgs/msg/String 'data: "Hello"' -1`

You should see the Ignition listener echoing the message.
