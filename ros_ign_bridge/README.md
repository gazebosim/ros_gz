# Bridge communication between ROS and Gazebo

This package provides a network bridge which enables the exchange of messages
between ROS and Gazebo Transport.

The following message types can be bridged for topics:

| ROS type                             | Gazebo type                            |
|--------------------------------------|:--------------------------------------:|
| builtin_interfaces/msg/Time          | ignition::msgs::Time                   |
| std_msgs/msg/Bool                    | ignition::msgs::Boolean                |
| std_msgs/msg/ColorRGBA               | ignition::msgs::Color                  |
| std_msgs/msg/Empty                   | ignition::msgs::Empty                  |
| std_msgs/msg/Float32                 | ignition::msgs::Float                  |
| std_msgs/msg/Float64                 | ignition::msgs::Double                 |
| std_msgs/msg/Header                  | ignition::msgs::Header                 |
| std_msgs/msg/Int32                   | ignition::msgs::Int32                  |
| std_msgs/msg/UInt32                  | ignition::msgs::UInt32                 |
| std_msgs/msg/String                  | ignition::msgs::StringMsg              |
| geometry_msgs/msg/Wrench             | ignition::msgs::Wrench                 |
| geometry_msgs/msg/Quaternion         | ignition::msgs::Quaternion             |
| geometry_msgs/msg/Vector3            | ignition::msgs::Vector3d               |
| geometry_msgs/msg/Point              | ignition::msgs::Vector3d               |
| geometry_msgs/msg/Pose               | ignition::msgs::Pose                   |
| geometry_msgs/msg/PoseWithCovariance | ignition::msgs::PoseWithCovariance     |
| geometry_msgs/msg/PoseStamped        | ignition::msgs::Pose                   |
| geometry_msgs/msg/Transform          | ignition::msgs::Pose                   |
| geometry_msgs/msg/TransformStamped   | ignition::msgs::Pose                   |
| geometry_msgs/msg/Twist              | ignition::msgs::Twist                  |
| geometry_msgs/msg/TwistWithCovariance| ignition::msgs::TwistWithCovariance    |
| mav_msgs/msg/Actuators (TODO)        | ignition::msgs::Actuators (TODO)       |
| nav_msgs/msg/Odometry                | ignition::msgs::Odometry               |
| nav_msgs/msg/Odometry                | ignition::msgs::OdometryWithCovariance |
| ros_ign_interfaces/msg/Contact       | ignition::msgs::Contact                |
| ros_ign_interfaces/msg/Contacts      | ignition::msgs::Contacts               |
| ros_ign_interfaces/msg/Entity        | ignition::msgs::Entity                 |
| ros_ign_interfaces/msg/GuiCamera     | ignition::msgs::GUICamera              |
| ros_ign_interfaces/msg/JointWrench   | ignition::msgs::JointWrench            |
| ros_ign_interfaces/msg/Light         | ignition::msgs::Light                  |
| ros_ign_interfaces/msg/StringVec     | ignition::msgs::StringMsg_V            |
| ros_ign_interfaces/msg/TrackVisual   | ignition::msgs::TrackVisual            |
| ros_ign_interfaces/msg/VideoRecord   | ignition::msgs::VideoRecord            |
| ros_ign_interfaces/msg/WorldControl  | ignition::msgs::WorldControl           |
| rosgraph_msgs/msg/Clock              | ignition::msgs::Clock                  |
| sensor_msgs/msg/BatteryState         | ignition::msgs::BatteryState           |
| sensor_msgs/msg/CameraInfo           | ignition::msgs::CameraInfo             |
| sensor_msgs/msg/FluidPressure        | ignition::msgs::FluidPressure          |
| sensor_msgs/msg/Imu                  | ignition::msgs::IMU                    |
| sensor_msgs/msg/Image                | ignition::msgs::Image                  |
| sensor_msgs/msg/JointState           | ignition::msgs::Model                  |
| sensor_msgs/msg/LaserScan            | ignition::msgs::LaserScan              |
| sensor_msgs/msg/MagneticField        | ignition::msgs::Magnetometer           |
| sensor_msgs/msg/PointCloud2          | ignition::msgs::PointCloudPacked       |
| tf2_msgs/msg/TFMessage               | ignition::msgs::Pose_V                 |
| trajectory_msgs/msg/JointTrajectory  | ignition::msgs::JointTrajectory        |

And the following for services:

| ROS type                             | Gazebo request             | Gazebo response       |
|--------------------------------------|:--------------------------:| --------------------- |
| ros_ign_interfaces/srv/ControlWorld  | ignition.msgs.WorldControl | ignition.msgs.Boolean |

Run `ros2 run ros_ign_bridge parameter_bridge -h` for instructions.

## Example 1a: Gazebo Transport talker and ROS 2 listener

Start the parameter bridge which will watch the specified topics.

```
# Shell A:
. ~/bridge_ws/install/setup.bash
ros2 run ros_ign_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
```

Now we start the ROS listener.

```
# Shell B:
. /opt/ros/galactic/setup.bash
ros2 topic echo /chatter
```

Now we start the Gazebo Transport talker.

```
# Shell C:
ign topic -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'
```

## Example 1b: ROS 2 talker and Gazebo Transport listener

Start the parameter bridge which will watch the specified topics.

```
# Shell A:
. ~/bridge_ws/install/setup.bash
ros2 run ros_ign_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
```

Now we start the Gazebo Transport listener.

```
# Shell B:
ign topic -e -t /chatter
```

Now we start the ROS talker.

```
# Shell C:
. /opt/ros/galactic/setup.bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hi'" --once
```

## Example 2: Run the bridge and exchange images

In this example, we're going to generate Gazebo Transport images using
Gazebo Sim, that will be converted into ROS images, and visualized with
`rqt_image_viewer`.

First we start Gazebo Sim (don't forget to hit play, or Gazebo Sim won't generate any images).

```
# Shell A:
ign gazebo sensors_demo.sdf
```

Let's see the topic where camera images are published.

```
# Shell B:
ign topic -l | grep image
/rgbd_camera/depth_image
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
. /opt/ros/galactic/setup.bash
ros2 run rqt_image_view rqt_image_view /rgbd_camera/image
```

You should see the current images in `rqt_image_view` which are coming from
Gazebo (published as Gazebo Msgs over Gazebo Transport).

The screenshot shows all the shell windows and their expected content
(it was taken using ROS 2 Galactic and Gazebo Fortress):

![Gazebo Transport images and ROS rqt](images/bridge_image_exchange.png)

## Example 3: Static bridge

In this example, we're going to run an executable that starts a bidirectional
bridge for a specific topic and message type. We'll use the `static_bridge`
executable that is installed with the bridge.

The example's code can be found under `ros_ign_bridge/src/static_bridge.cpp`.
In the code, it's possible to see how the bridge is hardcoded to bridge string
messages published on the `/chatter` topic.

Let's give it a try, starting with Gazebo -> ROS 2.

On terminal A, start the bridge:

`ros2 run ros_ign_bridge static_bridge`

On terminal B, we start a ROS 2 listener:

`ros2 topic echo /chatter std_msgs/msg/String`

And terminal C, publish an Gazebo message:

`ign topic -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'`

At this point, you should see the ROS 2 listener echoing the message.

Now let's try the other way around, ROS 2 -> Gazebo.

On terminal D, start an Igntion listener:

`ign topic -e -t /chatter`

And on terminal E, publish a ROS 2 message:

`ros2 topic pub /chatter std_msgs/msg/String 'data: "Hello"' -1`

You should see the Gazebo listener echoing the message.

## Example 4: Service bridge

It's possible to make ROS service requests into Gazebo. Let's try unpausing the simulation.

On terminal A, start the service bridge:

`ros2 run ros_ign_bridge parameter_bridge /world/shapes/control@ros_ign_interfaces/srv/ControlWorld`

On terminal B, start Gazebo, it will be paused by default:

`ign gazebo shapes.sdf`

On terminal C, make a ROS request to unpause simulation:

```
ros2 service call /world/<world_name>/control ros_ign_interfaces/srv/ControlWorld "world_control: pause: false"
```
