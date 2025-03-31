# Image utilities for using ROS and Gazebo Transport

This package provides a unidirectional bridge for images from Gazebo to ROS.
The bridge subscribes to Gazebo image messages (`ignition::msgs::Image`)
and republishes them to ROS using [image_transport](http://wiki.ros.org/image_transport).

For compressed images, install
[compressed_image_transport](http://wiki.ros.org/compressed_image_transport)
and the bridge will publish `/compressed` images. The same goes for other
`image_transport` plugins.
