# ROS + Ignition Gazebo

This package contains things that make it convenient to integrate ROS with
Ignition, such as:

* Launch files
* ROS-enabled executables

# Try it out

More usage examples can be seen on the
[ros_ign_gazebo_demos](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_gazebo_demos)
package.

## Run Ignition Gazebo

There's a convenient launch file, try for example:

    roslaunch ros_ign_gazebo ign_gazebo.launch ign_args:="shapes.sdf"

And you can directly call the executable, for example:

    rosrun ros_ign_gazebo ign_gazebo shapes.sdf

![](images/shapes_demo.png)


