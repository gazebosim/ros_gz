# ROS + Ignition Gazebo demos

This package contains demos showing how to use Ignition Gazebo with ROS.

## Run Ignition Gazebo

There's a convenient launch file, try for example:

    ros2 launch ros_ign_gazebo ign_gazebo.launch.py ign_args:="shapes.sdf"

![](images/shapes_demo.png)

## Air pressure

*TODO*: Pending bridge for `sensor_msgs/msg/FluidPressure`, [issue](https://github.com/osrf/ros_ign/issues/78).

Publishes fluid pressure readings.

    ros2 launch ros_ign_gazebo_demos air_pressure.launch.py

![](images/air_pressure_demo.png)

## Camera

Publishes RGB camera image and info.

Images can be exposed to ROS through `ros_ign_bridge` or `ros_ign_image`.

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    ros2 launch ros_ign_gazebo_demos image_bridge.launch.py

Using the regular bridge:

    ros2 launch ros_ign_gazebo_demos camera.launch.py

![](images/camera_demo.png)

## Diff drive

Send commands to a differential drive vehicle and listen to its odometry.

    ros2 launch ros_ign_gazebo_demos diff_drive.launch.py

Then unpause and send a command

    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"

![](images/diff_drive_demo.png)

## Depth camera

Depth camera data can be obtained as:

* `sensor_msgs/msg/Image`, through `ros_ign_bridge` or `ros_ign_image`
* `sensor_msgs/msg/PointCloud2`, through `ros_ign_point_cloud`

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    ros2 launch ros_ign_gazebo_demos image_bridge.launch.py

*TODO*: Blocked by `ros_ign_point_cloud` [issue](https://github.com/osrf/ros_ign/issues/40).

Using Ignition Gazebo plugin:

    ros2 launch ros_ign_gazebo_demos depth_camera.launch.py

![](images/depth_camera_demo.png)

## GPU lidar

GPU lidar data can be obtained as:

* `sensor_msgs/msg/LaserScan`, through the `ros_ign_bridge`
* `sensor_msgs/msg/PointCloud2`, through the `ros_ign_bridge` or `ros_ign_point_cloud`

Using the bridge:

    ros2 launch ros_ign_gazebo_demos gpu_lidar_bridge.launch.py

*TODO*: Blocked by `ros_ign_point_cloud` [issue](https://github.com/osrf/ros_ign/issues/40).

Using Ignition Gazebo plugin:

    ros2 launch ros_ign_gazebo_demos gpu_lidar.launch.py

![](images/gpu_lidar_demo.png)

## IMU

Publishes IMU readings.

    ros2 launch ros_ign_gazebo_demos imu.launch.py

![](images/imu_demo.png)

*TODO*: IMU display missing for RViz2

## Magnetometer

Publishes magnetic field readings.

    ros2 launch ros_ign_gazebo_demos magnetometer.launch.py

![](images/magnetometer_demo.png)

## RGBD camera

RGBD camera data can be obtained as:

* `sensor_msgs/msg/Image`, through `ros_ign_bridge` or `ros_ign_image`
* `sensor_msgs/msg/PointCloud2`, through `ros_ign_bridge` or `ros_ign_point_cloud`

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    ros2 launch ros_ign_gazebo_demos image_bridge.launch.py

Using the regular bridge:

    ros2 launch ros_ign_gazebo_demos rgbd_camera_bridge.launch.py

*TODO*: Blocked by `ros_ign_point_cloud` [issue](https://github.com/osrf/ros_ign/issues/40).

Using Ignition Gazebo plugin:

    ros2 launch ros_ign_gazebo_demos rgbd_camera.launch.py

![](images/rgbd_camera_demo.png)

## Battery

Get the current state of a battery.

    ros2 launch ros_ign_gazebo_demos battery.launch.py

Then send a command so the vehicle moves and drains the battery

    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"

![](images/battery_demo.png)
