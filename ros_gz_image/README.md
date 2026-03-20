# Image utilities for using ROS and Gazebo Transport

This package provides a unidirectional bridge for images from Gazebo to ROS.
The bridge subscribes to Gazebo image messages (`gz::msgs::Image`)
and republishes them to ROS using [image_transport](http://wiki.ros.org/image_transport).

For compressed images, install
[compressed_image_transport](http://wiki.ros.org/compressed_image_transport)
and the bridge will publish `/compressed` images. The same goes for other
`image_transport` plugins.

To run the bridge from the command line:

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2
```

## Parameters

### `qos` (string, default: `"default"`)

Modify the [Quality of Service (QoS) policy](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-policies) used to publish images. Accepted values: `default`, `sensor_data`, `system_default`.

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args -p qos:=sensor_data
```

### `lazy` (bool, default: `false`)

When set to `true`, the bridge will only subscribe to the Gazebo topic when there is at least one ROS subscriber. This avoids unnecessary message processing when no one is listening.

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args -p lazy:=true
```

### `subscription_heartbeat` (int, default: `1000`)

Interval in milliseconds at which the bridge checks the ROS subscriber count to start or stop the Gazebo subscription. Only relevant when `lazy:=true`.

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args -p lazy:=true -p subscription_heartbeat:=500
```
