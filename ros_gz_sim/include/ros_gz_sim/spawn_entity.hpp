// Copyright 2025 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file spawn_entity.hpp
/// \brief Defines utilities and a ROS 2 node for spawning entities in a Gazebo simulation.

#ifndef ROS_GZ_SIM__SPAWN_ENTITY_HPP_
#define ROS_GZ_SIM__SPAWN_ENTITY_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

/// \brief Struct representing parsed command-line arguments for entity
/// spawning.
struct CommandLineArgs
{
  std::string model_name;           ///< Model name to be used in Gazebo.
  std::string sdf_filename;         ///< Path to the SDF model file.
  std::vector<double> position;     ///< XYZ position of the model.
  std::vector<double> quaternion;   ///< Orientation as quaternion [x, y, z, w].
  std::vector<double> euler;        ///< Orientation as Euler angles [roll, pitch, yaw].
};

/// \brief Parses command-line arguments for spawning an entity.
/// \param[in] argc Argument count.
/// \param[in] argv Argument vector.
/// \return Struct containing parsed arguments.
CommandLineArgs parse_arguments(int argc, char **argv);

/// \brief A ROS 2 node for spawning entities into a Gazebo simulation.
class EntitySpawner : public rclcpp::Node {
public:
  /// \brief Default constructor. Initializes the node and spawn client.
  EntitySpawner();

  /// \brief Constructor that accepts an external spawn service client.
  /// \param[in] client Shared pointer to the spawn entity service
  /// client.
  explicit EntitySpawner(
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client);

  /// \brief Spawns a model entity in the simulation.
  /// \param[in] model_name Name of the model.
  /// \param[in] sdf_filename Path to the SDF file.
  /// \param[in] pose Initial pose of the model.
  /// \return True if the model was spawned successfully, false otherwise.
  bool spawn_entity(
    const std::string & model_name,
    const std::string & sdf_filename,
    const geometry_msgs::msg::Pose & pose);

protected:
  /// \brief Client used to call the spawn entity service.
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;
};

#endif  // ROS_GZ_SIM__SPAWN_ENTITY_HPP_
