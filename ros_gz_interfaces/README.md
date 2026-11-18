# Message and service data structures for interacting with Gazebo from ROS2

This package contains Gazebo-specific ROS message and service data structures (.msg and .srv).
Most messages mirror a Gazebo message, linked below, and can be bridged with `ros_gz_bridge`.

## Messages (.msg)

* [AirSpeed](msg/AirSpeed.msg): related to [gz::msgs::AirSpeed](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/air_speed.proto). Air speed sensor reading.
* [Altimeter](msg/Altimeter.msg): related to [gz::msgs::Altimeter](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/altimeter.proto). Altimeter sensor reading.
* [Contact](msg/Contact.msg): related to [gz::msgs::Contact](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/contact.proto). Contact info between collisions in Gazebo Sim.
* [Contacts](msg/Contacts.msg): related to [gz::msgs::Contacts](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/contacts.proto). A list of contacts.
* [Dataframe](msg/Dataframe.msg): related to [gz::msgs::Dataframe](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/dataframe.proto). Payload with sender and destination addresses, used by the Gazebo comms system.
* [Entity](msg/Entity.msg): related to [gz::msgs::Entity](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/entity.proto). Entity of Gazebo Sim.
* [EntityFactory](msg/EntityFactory.msg): related to [gz::msgs::EntityFactory](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/entity_factory.proto). Message to create a new entity.
* [EntityWrench](msg/EntityWrench.msg): related to [gz::msgs::EntityWrench](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/entity_wrench.proto). Wrench to be applied to a specified Entity of Gazebo Sim.
* [Float32Array](msg/Float32Array.msg): related to [gz::msgs::Float_V](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/float_v.proto). An array of floats.
* [GuiCamera](msg/GuiCamera.msg): related to [gz::msgs::GUICamera](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/gui_camera.proto). Pose and settings of the GUI camera.
* [JointWrench](msg/JointWrench.msg): related to [gz::msgs::JointWrench](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/joint_wrench.proto). Forces and torques between the two bodies of a joint.
* [Light](msg/Light.msg): related to [gz::msgs::Light](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/light.proto). Light info in Gazebo Sim.
* [LogicalCameraImage](msg/LogicalCameraImage.msg): related to [gz::msgs::LogicalCameraImage](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/logical_camera_image.proto). Models seen by a logical camera.
* [LogicalCameraImageModel](msg/LogicalCameraImageModel.msg): a model in a `LogicalCameraImage`, with its name and pose.
* [LogPlaybackStatistics](msg/LogPlaybackStatistics.msg): related to [gz::msgs::LogPlaybackStatistics](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/log_playback_stats.proto). Statistics about a log during playback.
* [MaterialColor](msg/MaterialColor.msg): related to [gz::msgs::MaterialColor](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/material_color.proto). Material colors to apply to an entity.
* [ParamVec](msg/ParamVec.msg): related to [gz::msgs::Param](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/param.proto). A set of name and value parameters.
* [SensorNoise](msg/SensorNoise.msg): related to [gz::msgs::SensorNoise](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/sensor_noise.proto). Noise model of a sensor.
* [StringVec](msg/StringVec.msg): related to [gz::msgs::StringMsg_V](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/stringmsg_v.proto). A list of strings.
* [TrackVisual](msg/TrackVisual.msg): related to [gz::msgs::TrackVisual](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/track_visual.proto). Settings for a camera that tracks a visual.
* [VideoRecord](msg/VideoRecord.msg): related to [gz::msgs::VideoRecord](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/video_record.proto). Starts and stops video recording.
* [WorldControl](msg/WorldControl.msg): related to [gz::msgs::WorldControl](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/world_control.proto). Message to control world of Gazebo Sim.
* [WorldReset](msg/WorldReset.msg): related to [gz::msgs::WorldReset](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/world_reset.proto). Reset time and model of simulation.
* [WorldStatistics](msg/WorldStatistics.msg): related to [gz::msgs::WorldStatistics](https://github.com/gazebosim/gz-msgs/blob/gz-msgs12/proto/gz/msgs/world_stats.proto). World statistics of Gazebo Sim.

## Services (.srv)

* [ControlWorld](srv/ControlWorld.srv): Control world of Gazebo Sim, for example pause, pause after multiple steps, resume, etc.
* [DeleteEntity](srv/DeleteEntity.srv): Delete Entity in Gazebo Sim
* [SetEntityPose](srv/SetEntityPose.srv): Set pose of Entity in Gazebo Sim
* [SpawnEntity](srv/SpawnEntity.srv): Spawn a Entity in Gazebo Sim
