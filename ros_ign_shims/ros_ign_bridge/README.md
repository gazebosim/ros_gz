# ROS-Gazebo packages

`ros_ign_bridge` is a shim metapackage for `ros_gz_bridge`.
It installs symlinks for CMake targets and executables, as well as for the package directory.
This allows for use of `ros_gz_bridge` with the old `ros_ign_bridge` ROS APIs.

It is deprecated and will be removed, please use `ros_gz_bridge` instead!
