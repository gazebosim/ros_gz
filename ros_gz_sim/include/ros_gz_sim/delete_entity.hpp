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

/// \file delete_entity.hpp
/// \brief Defines utilities and a ROS 2 node for deleting entities from a Gazebo simulation.

#ifndef ROS_GZ_SIM__DELETE_ENTITY_HPP_
#define ROS_GZ_SIM__DELETE_ENTITY_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>

/// \brief A ROS 2 node for deleting entities from a Gazebo simulation.
class EntityDeleter : public rclcpp::Node {
public:
  /// \brief Default constructor. Initializes the node and delete entity client.
  EntityDeleter();

  /// \brief Constructor that accepts an external delete service client.
  /// \param[in] client Shared pointer to the delete entity service client.
  explicit EntityDeleter(
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr client);

  /// \brief Deletes an entity from the simulation.
  /// \param[in] entity_name Name of the entity.
  /// \param[in] entity_id ID of the entity.
  /// \param[in] entity_type Type of the entity.
  /// \return True if the entity was deleted successfully, false otherwise.
  bool delete_entity(
    const std::string & entity_name, int entity_id,
    int entity_type);

protected:
  /// \brief Client used to call the delete entity service.
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr client_;
};

#endif  // ROS_GZ_SIM__DELETE_ENTITY_HPP_
