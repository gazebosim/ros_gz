# ROS + Ignition Gazebo

This package contains things that make it convenient to integrate ROS with Ignition, such as:

 - Launch files
 - ROS-enabled executables

### Run Ignition Gazebo

There's a convenient launch file, try for example:

```bash
ros2 launch ros_ign_gazebo ign_gazebo.launch.py ign_args:="shapes.sdf"
```

### Spawn entities

The `create` executable can be used to spawn SDF or URDF entities from:

 - A file on disk or from Ignition Fuel
 - A ROS parameter

For example, start Ignition Gazebo:

```
ros2 launch ros_ign_gazebo ign_gazebo.launch.py
```

then spawn a model:

```
ros2 run ros_ign_gazebo create -world default -file 'https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Gazebo'
```

See more options with:

```
ros2 run ros_ign_gazebo create --helpshort
```
