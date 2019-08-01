# ROS 1 + Ignition Gazebo demos

This package contains demos showing how to use Ignition Gazebo with ROS 1.

## Run Ignition Gazebo

There's a convenient launch file, try for example:

    roslaunch ros1_ign_gazebo_demos ign_gazebo.launch args:="shapes.sdf"

And you can directly call the executable, for example:

    rosrun ros1_ign_gazebo_demos ign_gazebo shapes.sdf

![](images/shapes_demo.png)

## Air pressure

Publishes fluid pressure readings.

    roslaunch ros1_ign_gazebo_demos air_pressure.launch

![](images/air_pressure_demo.png)

## Camera

Publishes RGB camera image and info.

Images can be exposed to ROS through `ros1_ign_bridge` or `ros1_ign_image`.

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    roslaunch ros1_ign_gazebo_demos image_bridge.launch

Using the regular bridge:

    roslaunch ros1_ign_gazebo_demos camera.launch

![](images/camera_demo.png)

## Diff drive

Send commands to a differential drive vehicle and listen to its odometry.

    roslaunch ros1_ign_gazebo_demos diff_drive.launch

Then send a command

    rostopic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"

![](images/diff_drive_demo.png)

## Depth camera

Depth camera data can be obtained as:

* `sensor_msgs/Image`, through `ros1_ign_bridge` or `ros1_ign_image`
* `sendor_msgs/PointCloud2`, through `ros1_ign_bridge` or `ros1_ign_point_cloud`

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    roslaunch ros1_ign_gazebo_demos image_bridge.launch

Using the regular bridge:

    roslaunch ros1_ign_gazebo_demos depth_camera_bridge.launch

Using Ignition Gazebo plugin:

    roslaunch ros1_ign_gazebo_demos depth_camera.launch

![](images/depth_camera_demo.png)

## GPU lidar

GPU lidar data can be obtained as:

* `sensor_msgs/LaserScan`, through the `ros1_ign_bridge`
* `sendor_msgs/PointCloud2`, through the `ros1_ign_bridge` or `ros1_ign_point_cloud`

Using the bridge:

    roslaunch ros1_ign_gazebo_demos gpu_lidar_bridge.launch

Using Ignition Gazebo plugin:

    roslaunch ros1_ign_gazebo_demos gpu_lidar.launch

![](images/gpu_lidar_demo.png)

## IMU

Publishes IMU readings.

    roslaunch ros1_ign_gazebo_demos imu.launch

![](images/imu_demo.png)

## Magnetometer

Publishes magnetic field readings.

    roslaunch ros1_ign_gazebo_demos magnetometer.launch

![](images/magnetometer_demo.png)

## RGBD camera

RGBD camera data can be obtained as:

* `sensor_msgs/Image`, through `ros1_ign_bridge` or `ros1_ign_image`
* `sendor_msgs/PointCloud2`, through `ros1_ign_bridge` or `ros1_ign_point_cloud`

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    roslaunch ros1_ign_gazebo_demos image_bridge.launch

Using the regular bridge:

    roslaunch ros1_ign_gazebo_demos rgbd_camera_bridge.launch

Using Ignition Gazebo plugin:

    roslaunch ros1_ign_gazebo_demos rgbd_camera.launch

![](images/rgbd_camera_demo.png)

## Battery

Get the current state of a battery.

    roslaunch ros1_ign_gazebo_demos battery.launch

Then send a command so the vehicle moves and drains the battery

    rostopic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"

![](images/battery_demo.png)
