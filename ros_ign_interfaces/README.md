# Message and service data structures for interacting with Ignition from ROS2

This package currently contains some Ignition-specific ROS message and service data structures (.msg and .srv)

## Messages (.msg)

* [Contact](msg/Contact.msg): related to [ignition::msgs::Contact](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/contact.proto). Contant info bewteen collisions in Ignition Gazebo.
* [Contacts](msg/Contacts.msg): related to [ignition::msgs::Contacts](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/contacts.proto). A list of contacts.
* [Entity](msg/Entity.msg): related to [ignition::msgs::Entity](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/entity.proto). Entity of Ignition Gazebo.
* [EntityFactory](msg/EntityFactory.msg): related to [ignition::msgs::EntityFactory](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/entity_factory.proto). Message to create a new entity.
* [Light](msg/Light.msg): related to [ignition::msgs::Light](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/light.proto). Light info in Ignition Gazebo.
* [WorldControl](msg/WorldControl.msg): related to [ignition::msgs::WorldControl](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/world_control.proto). Message to control world of Ignition Gazebo.
* [WorldReset](msg/WorldReset.msg): related to [ignition::msgs::WorldReset](https://github.com/ignitionrobotics/ign-msgs/blob/ign-msgs7/proto/ignition/msgs/world_reset.proto). Reset time and model of simulation.

## Services (.srv)

* [ControlWorld](srv/ControlWorld.srv): Control world of Ignition Gazebo,for example,pasue,pasue with multiple steps,resume,etc.
* [DeleteEntity](srv/DeleteEntity.srv): Delete Entity in Ignition Gazebo
* [SetEntityPose](srv/SetEntityPose.srv): Set pose of Entity in Ignition Gazebo
* [SpawnEntity](srv/SpawnEntity.srv): Spawn a Entity in Ignition Gazebo
