# ROS + Gazebo Sim demos

This package contains demos showing how to use Gazebo Sim with ROS.

## Run Gazebo Sim

There's a convenient launch file, try for example:

    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"

![](images/shapes_demo.png)

## Air pressure

Publishes fluid pressure readings.

    ros2 launch ros_gz_sim_demos air_pressure.launch.xml

This demo also shows the use of custom QoS parameters. The sensor data is
published as as "best-effort", so trying to subscribe to "reliable" data won't
work. See the difference between:

    ros2 topic echo /air_pressure --qos-reliability best_effort

And

    ros2 topic echo /air_pressure --qos-reliability reliable

![](images/air_pressure_demo.png)

## Battery

Get the current state of a battery.

    ros2 launch ros_gz_sim_demos battery.launch.py

Then send a command so the vehicle moves and drains the battery.

    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"

![](images/battery_demo.png)

## Camera

Publishes RGB camera image and info.

Images can be exposed to ROS through `ros_gz_bridge` or `ros_gz_image`.

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    ros2 launch ros_gz_sim_demos image_bridge.launch.py

Using the regular bridge:

    ros2 launch ros_gz_sim_demos camera.launch.xml

To use a camera that only publishes information when triggered:

    ros2 launch ros_gz_sim_demos triggered_camera.launch.xml

Trigger the camera:

    ros2 topic pub /camera/trigger std_msgs/msg/Bool "{data: true}" --once

![](images/camera_demo.png)

## Diff drive

Send commands to a differential drive vehicle and listen to its odometry.

    ros2 launch ros_gz_sim_demos diff_drive.launch.xml

Then send a command

    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"

This demo also shows the use of custom QoS parameters. The commands are
subscribed to as "reliable", so trying to publish "best-effort" commands
won't work. See the difference between:

    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.0}}" --qos-reliability reliable

And

    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.0}}" --qos-reliability best_effort

![](images/diff_drive_demo.png)

## Depth camera

Depth camera data can be obtained as:

* `sensor_msgs/msg/Image`, through `ros_gz_bridge` or `ros_gz_image`
* `sensor_msgs/msg/PointCloud2`, through `ros_gz_point_cloud`

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/depth_camera

*TODO*: Blocked by `ros_gz_point_cloud` [issue](https://github.com/gazebosim/ros_gz/issues/40).

Using Gazebo Sim plugin:

    ros2 launch ros_gz_sim_demos depth_camera.launch.py

![](images/depth_camera_demo.png)

## GPU lidar

GPU lidar data can be obtained as:

* `sensor_msgs/msg/LaserScan`, through the `ros_gz_bridge`
* `sensor_msgs/msg/PointCloud2`, through the `ros_gz_bridge` or `ros_gz_point_cloud`

Using the bridge:

    ros2 launch ros_gz_sim_demos gpu_lidar_bridge.launch.xml

*TODO*: Blocked by `ros_gz_point_cloud` [issue](https://github.com/gazebosim/ros_gz/issues/40).

Using Gazebo Sim plugin:

    ros2 launch ros_gz_sim_demos gpu_lidar.launch.py

![](images/gpu_lidar_demo.png)

## IMU

Publishes IMU readings.

    ros2 launch ros_gz_sim_demos imu.launch.xml

![](images/imu_demo.png)

*TODO*: IMU display missing for RViz2

## Magnetometer

Publishes magnetic field readings.

    ros2 launch ros_gz_sim_demos magnetometer.launch.xml

![](images/magnetometer_demo.png)

## GNSS

Publishes satellite navigation readings, only available in Fortress on.

GNSS information can be obtained as:

    # sensor_msgs/msg/NavSatFix
    ros2 launch ros_gz_sim_demos navsat.launch.xml
    # gps_msgs/msg/GPSFix
    ros2 launch ros_gz_sim_demos navsat_gpsfix.launch.xml

![](images/navsat_demo.png)

## RGBD camera

RGBD camera data can be obtained as:

* `sensor_msgs/msg/Image`, through `ros_gz_bridge` or `ros_gz_image`
* `sensor_msgs/msg/PointCloud2`, through `ros_gz_bridge` or `ros_gz_point_cloud`

Using the image bridge (unidirectional, uses [image_transport](http://wiki.ros.org/image_transport)):

    # RGB image
    ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/rgbd_camera/image
    # Depth image
    ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/rgbd_camera/depth_image

Using the regular bridge:

    ros2 launch ros_gz_sim_demos rgbd_camera_bridge.launch.xml

*TODO*: Blocked by `ros_gz_point_cloud` [issue](https://github.com/gazebosim/ros_gz/issues/40).

Using Gazebo Sim plugin:

    ros2 launch ros_gz_sim_demos rgbd_camera.launch.py

![](images/rgbd_camera_demo.png)

## Robot description publisher

Leverage the robot description publisher to spawn a new urdf model in gazebo and
show it in rviz2.
To try the demo launch:

    ros2 launch ros_gz_sim_demos robot_description_publisher.launch.py

![](images/robot_state_publisher_demo.png)

## Joint States Publisher

Publishes joint states of the robot.

To try the demo launch:

    ros2 launch ros_gz_sim_demos joint_states.launch.py

![](images/joint_states.png)

## Multi robot

The `multi_robot` demo shows how to start multiple robots from the same robot SDF file, with separate ROS and Gazebo topics for each robot namespace.

To try the demo launch:

```bash
ros2 launch ros_gz_sim_demos multi_robot.launch.py
```

The demo can be used as a reference for different ways to start multiple robots.

### 1. Define multiple robots in the SDF

The `multi_robot.sdf` world defines two robots from the same vehicle model. There are two supported styles.

* Wrap the included model in a `<model>` tag and set the namespace on the wrapper model:

  ```xml
  <model name="vehicle" namespace="robot1">
    <self_collide>true</self_collide>
    <pose>0 0 1 0 0 0</pose>
    <include merge="true">
      <uri>package://ros_gz_sim_demos/models/vehicle</uri>
    </include>
  </model>
  ```

* Include the same model directly and set both the model name and namespace in the `<include>` block:

  ```xml
  <include>
    <uri>package://ros_gz_sim_demos/models/vehicle</uri>
    <name>robot2</name>
    <namespace>__name__</namespace>
    <pose>0 2 1 0 0 0</pose>
  </include>
  ```

  In this example, `__name__` will be resolved to the model name, so the second robot will use the `robot2` namespace.

### 2. Spawn a namespaced robot with `ros_gz_sim create` node

The `ros_gz_sim create` executable can spawn robots and pass the namespace with `-ns`.

If `-ns` is not provided, the namespace behavior follows the source SDF file. Use `-ns` only when you want to explicitly override the namespace at spawn time.

There are two common ways to use it:
* Use it from a launch file. 
  The `multi_robot.launch.xml` demo uses `ros_gz_sim create` to spawn `robot3` from the same `vehicle_sdf` file and pass a namespace with `-ns`:

  ```xml
  <node
    pkg="ros_gz_sim"
    exec="create"
    args="-world multi_robot
          -file $(find-pkg-share ros_gz_sim_demos)/models/vehicle/model.sdf
          -name robot3
          -ns __name__
          -x 0.0
          -y 4.0
          -z 1.0"
    output="screen" />
  ```

* Use it from the command line:

  ```bash
  export VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"
  ros2 run ros_gz_sim create \
      -world multi_robot \
      -file "$VEHICLE_SDF" \
      -name robot5 \
      -ns robot5 \
      -x 0.0 \
      -y 8.0 \
      -z 1.0
  ```

### 3. Spawn a namespaced robot using the launch file included in `ros_gz_sim`

The `gz_spawn_model.launch.py` launch file can spawn robots and pass the namespace with `entity_namespace`.

If `entity_namespace` is not provided, the namespace behavior follows the source SDF file. Use `entity_namespace` only when you want to explicitly override the namespace at spawn time.

There are two common ways to use it:
* Use the `gz_spawn_model` action from the launch file.
    The`multi_robot.launch.xml` demo uses `gz_spawn_model` to spawn `robot4` from the same `vehicle_sdf` file and pass a namespace with `entity_namespace`:

  ```xml
  <gz_spawn_model
    world="multi_robot"
    file="$(find-pkg-share ros_gz_sim_demos)/models/vehicle/model.sdf"
    model_string=""
    topic=""
    entity_name="robot4"
    entity_namespace="__name__"
    allow_renaming="false"
    x="0.0"
    y="6.0"
    z="1.0"
    roll="0.0"
    pitch="0.0"
    yaw="0.0">
  </gz_spawn_model>
  ```

* Launch `gz_spawn_model.launch.py` directly from the command line:

  ```bash
  export VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"
  ros2 launch ros_gz_sim gz_spawn_model.launch.py \
      world:=multi_robot \
      file:="$VEHICLE_SDF" \
      entity_name:=robot6 \
      entity_namespace:=__name__ \
      x:=0.0 \
      y:=10.0 \
      z:=1.0
  ```
### 4. Spawn a namespaced robot with ROS 2 Simulation Interfaces
  The ROS 2 Simulation Interfaces provide ROS 2 services for controlling and interacting with simulation environments.

  The `/gzserver/spawn_entity` service can spawn robots and pass the namespace with `entity_namespace`.

  If `entity_namespace` is not provided, the namespace behavior follows the source SDF file. Use `entity_namespace` only when you want to explicitly override the namespace at spawn time.

  ``` bash
  export VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"
  ros2 service call /gzserver/spawn_entity simulation_interfaces/srv/SpawnEntity "{
    name: 'robot7',
    entity_resource: {
      uri: "$VEHICLE_SDF"},
    entity_namespace: ['__name__'],
    allow_renaming: false,
    initial_pose: {
      pose: {
        position: {x: 0.0, y: 12.0, z: 1.0},
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      }
    }
  }"
  ```

### 5. Spawn a namespaced robot with `gz service`

The Gazebo `create` service can also spawn robots from the same SDF file into the running `multi_robot` world. The model name and namespace can be set in the request.

If `namespace: {data: ...}` is not provided, the namespace behavior follows the source SDF file. Use it only when you want to explicitly set or override the namespace at spawn time.

```bash
export VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"
gz service -s /world/multi_robot/create_with_ns/blocking \
    --reqtype gz.msgs.EntityFactoryWithNs \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'sdf_filename: "'"$VEHICLE_SDF"'",
           name: "robot8",
           namespace: {data: "robot8"},
           pose: {
             position: {x: 0.0, y: 14.0, z: 1.0},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
           }'
```

## Bridging joint state and pose publishers

The launch file demonstrates bridging Gazebo poses to TFMessage to visualize the pose
and transforms of a robot in rviz.

To try the demo launch:

    ros2 launch ros_gz_sim_demos tf_bridge.launch.xml

![](images/tf_bridge.gif)

## Managing Entities

The `ros_gz_sim` package provides a set of utilities for managing entities (models, lights, links, etc.) in Gazebo simulations through ROS 2.
This package enables seamless communication between ROS 2 and Gazebo, allowing you to:

- **Spawn entities**: Add new models and objects to a running Gazebo simulation
- **Set entity poses**: Dynamically adjust the position and orientation of existing entities
- **Delete entities**: Remove entities from the simulation environment

### Launching Gazebo

```bash
gz sim -v 4 ~/ros2_ws/src/ros_gz/ros_gz_sim_demos/worlds/default.sdf
```

### 1. Spawning Entities

Spawn new entities into the simulation:

1. Run the ROS-Gazebo bridge for the spawn service:

```bash
ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/SpawnEntity
```

2. Spawn your entity:

```bash
ros2 run ros_gz_sim spawn_entity --name <model_name> --sdf_filename <path_to_sdf_file> [--pos x y z] [--quat x y z w | --euler roll pitch yaw]
```

**Example:**

```bash
ros2 run ros_gz_sim spawn_entity --name cardboard_box --sdf_filename $(ros2 pkg prefix ros_gz_sim_demos)/share/ros_gz_sim_demos/models/cardboard_box/model.sdf --pos 1.0 2.0 0.5 --euler 0.0 0.0 1.57
```

or

```bash
ros2 run ros_gz_sim spawn_entity --name cardboard_box --sdf_filename /full/path/to/ros_gz_ws/src/ros_gz_sim_demos/models/cardboard_box/model.sdf --pos 1.0 2.0 0.5 --euler 0.0 0.0 1.57
```

![spawn_entity](resources/spawn.gif)

### 2. Setting Entity Poses

Dynamically adjust the position and orientation of existing entities:

1. Run the ROS-Gazebo bridge for the set pose service:

```bash
ros2 run ros_gz_bridge parameter_bridge /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

2. Set the entity's pose:

```bash
ros2 run ros_gz_sim set_entity_pose [--name NAME | --id ID] [--type TYPE] [--pos X Y Z] [--quat X Y Z W | --euler ROLL PITCH YAW]
```

**Examples:**

Using entity name with Euler angles for rotation:

```bash
ros2 run ros_gz_sim set_entity_pose --name cardboard_box --pos 3.0 4.0 1.0 --euler 0.0 0.0 1.57
```

Using entity ID with quaternion for rotation:

```bash
ros2 run ros_gz_sim set_entity_pose --id 8 --pos 3.0 4.0 1.0 --quat 0.0 0.0 0.7071 0.7071
```

![set_entity](resources/set_entity.gif)

### 3. Deleting Entities

Remove entities from the simulation:

1. Run the ROS-Gazebo bridge for the delete service:

```bash
ros2 run ros_gz_bridge parameter_bridge /world/default/remove@ros_gz_interfaces/srv/DeleteEntity
```

2. Delete the entity:

```bash
ros2 run ros_gz_sim delete_entity [--name NAME | --id ID] [--type TYPE]
```

**Examples:**

Using entity name:

```bash
ros2 run ros_gz_sim delete_entity --name cardboard_box
```

Using entity ID:

```bash
ros2 run ros_gz_sim delete_entity --id 8
```

Using a specific entity type:

```bash
ros2 run ros_gz_sim delete_entity --name cardboard_box --type 2
```

![delete_entity](resources/delete_entity.gif)

### Entity Type Reference

When using the `set_entity_pose` and `delete_entity` commands, you can specify the entity type using the `--type` flag. The following type values are available:

| Value | Entity Type |
|-------|-------------|
| 0     | NONE        |
| 1     | LIGHT       |
| 2     | LINK        |
| 3     | VISUAL      |
| 4     | COLLISION   |
| 5     | SENSOR      |
| 6     | MODEL (default) |
