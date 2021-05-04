# Message and service data structures for interacting with Ignition from ROS2

This is migration of [gazebo_ros_pkgs/gazebo_msgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/foxy/gazebo_msgs) for Ignition.

This package currently contains some Ignition-specific ROS message and service data structures (.msg and .srv)

## Messages (.msg)

* [ContactState](msg/ContactState.msg) : Contact Info In Ignition Gazebo 
* [ContactStates](msg/ContactStates.msg) : An array of ContactState


## Services (.srv)

* [ControlWorld](srv/ControlWorld.srv) : Control world of Ignition Gazebo,for example,pasue,pasue with multiple steps,resume,etc.
* [DeleteEntity](srv/DeleteEntity.srv) : Delete Entity in Ignition Gazebo
* [SetEntityPose](srv/SetEntityPose.srv) : Set pose of Entity in Ignition Gazebo
* [SpawnEntity](srv/SpawnEntity.srv) : Spawn a Entity in Ignition Gazebo

