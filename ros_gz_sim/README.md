# ROS + Gazebo Sim

This package contains things that make it convenient to integrate ROS with Gazebo, such as:

 - Launch files
 - ROS-enabled executables

### Run Gazebo Sim

There's a convenient launch file, try for example:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"
```

### Spawn entities

The `create` executable can be used to spawn SDF or URDF entities from:

 - A file on disk or from Gazebo Fuel
 - A ROS parameter

For example, start Gazebo Sim:

```
ros2 launch ros_gz_sim gz_sim.launch.py
```

then spawn a model:

```
ros2 run ros_gz_sim create -world default -file 'https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Gazebo'
```

See more options with:

```
ros2 run ros_gz_sim create --helpshort
```
