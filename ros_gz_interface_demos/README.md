# Gazebo Entity Manager Documentation

This documentation provides setup and usage instructions for the Gazebo Entity Manager package (`ros_gz_interface_demos`), which allows you to spawn, position, and delete entities in Gazebo simulations through ROS 2.

## Overview

The Gazebo Entity Manager is a C++ package that provides a set of ROS 2 utilities for managing entities (models, lights, links, etc.) in a Gazebo simulation. These utilities allow you to:

1. Spawn new entities into the simulation
2. Modify the pose (position and orientation) of existing entities
3. Delete entities from the simulation

## Prerequisites

- ROS 2 (tested with Rolling/Humble)
- Gazebo Harmonic/Ionic
- ROS-Gazebo bridge package (`ros_gz_bridge`)

## Installation

1. Create a ROS 2 workspace if you don't have one:
   ```bash
   mkdir -p ~/gz_ws/src
   cd ~/gz_ws/src
   ```

2. Clone the package into your workspace:
   ```bash
   git clone https://github.com/khaledgabr77/gz_entity_manager_cpp.git
   ```

3. Build the workspace:
   ```bash
   cd ~/gz_ws
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source ~/gz_ws/install/setup.bash
   ```

## Usage

The package provides three main executables:
- `spawn_entity`: Creates new entities in the Gazebo simulation
- `set_entity_pose`: Modifies the position and orientation of existing entities
- `delete_entity`: Removes entities from the simulation

### 1. Spawning Entities

To spawn an entity into the simulation:

1. Launch Gazebo with your world file:
   ```bash
   gz sim /path/to/your/world.sdf
   ```

2. Run the ROS-Gazebo bridge for the spawn service:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/SpawnEntity
   ```

3. Spawn your entity using the `spawn_entity` executable:
   ```bash
   ros2 run ros_gz_interface_demos spawn_entity <model_name> <path_to_model_sdf>
   ```

   Example:
   ```bash
   ros2 run ros_gz_interface_demos spawn_entity cardboard_box /home/gz_ws/src/ros_gz_interface_demos/models/cardboard_box/model.sdf
   ```

### 2. Setting Entity Pose

To modify the pose (position and orientation) of an existing entity:

1. Launch Gazebo with your world file:
   ```bash
   gz sim /path/to/your/world.sdf
   ```

2. Run the ROS-Gazebo bridge for the set pose service:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose
   ```

3. Modify the entity's pose using the `set_entity_pose` executable:
   ```bash
   ros2 run ros_gz_interface_demos set_entity_pose [--name NAME | --id ID] [--type TYPE] [--pos X Y Z] [--quat X Y Z W | --euler ROLL PITCH YAW]
   ```

   Example (using entity name and Euler angles for rotation):
   ```bash
   ros2 run ros_gz_interface_demos set_entity_pose --name cardboard_box --pos 1.0 2.0 3.0 --euler 0.0 0.0 1.57
   ```

   Example (using entity ID and quaternion for rotation):
   ```bash
   ros2 run ros_gz_interface_demos set_entity_pose --id 8 --pos 1.0 2.0 3.0 --quat 0.0 0.0 0.0 1.0
   ```

### 3. Deleting Entities

To delete an entity from the simulation:

1. Launch Gazebo with your world file:
   ```bash
   gz sim /path/to/your/world.sdf
   ```

2. Run the ROS-Gazebo bridge for the delete service:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /world/default/remove@ros_gz_interfaces/srv/DeleteEntity
   ```

3. Delete the entity using the `delete_entity` executable:
   ```bash
   ros2 run ros_gz_interface_demos delete_entity [--name NAME | --id ID] [--type TYPE]
   ```

   Example (using entity name):
   ```bash
   ros2 run ros_gz_interface_demos delete_entity --name cardboard_box
   ```

   Example (using entity ID):
   ```bash
   ros2 run ros_gz_interface_demos delete_entity --id 8
   ```

   Example (specifying entity type - in this case, a LINK):
   ```bash
   ros2 run ros_gz_interface_demos delete_entity --name cardboard_box --type 2
   ```

## Entity Type Reference

When using the `set_entity_pose` and `delete_entity` commands, you can specify the entity type using the `--type` flag. The following type values are available:

| Value | Type       |
|-------|------------|
| 0     | NONE       |
| 1     | LIGHT      |
| 2     | LINK       |
| 3     | VISUAL     |
| 4     | COLLISION  |
| 5     | SENSOR     |
| 6     | MODEL (default) |

## Troubleshooting

1. **Service not available**: If you see "Service not available, waiting..." messages, ensure that:
   - Gazebo is running with the correct world
   - The ROS-Gazebo bridge is running for the correct service
   - The world name in your service path matches your Gazebo world name (default: "default")

2. **Failed to spawn/delete/set pose**: Check that:
   - The entity name or ID is correct
   - The entity type is correct (if specified)
   - The SDF file path is valid (for spawn_entity)

3. **File not found errors**: Ensure you're using absolute paths or properly referenced relative paths for model SDF files.

## Advanced Usage

### Launching Complete Simulation

For convenience, you can create launch files that start Gazebo, the ROS-Gazebo bridges, and your desired entity management commands together. Example launch files are provided in the `launch` directory of the package.

### Integration with ROS 2 Applications

You can use these utilities as part of larger ROS 2 applications by either:
- Calling the executables from your own launch files or scripts
- Using the provided C++ classes (EntitySpawner, EntityPoseSetter, EntityDeleter) in your own nodes

## License

This package is licensed under the Apache License 2.0.

## Contact

For questions or issues, please contact Khaled Gabr at khaledgabr77@email.com.