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

/// \file set_entity_pose.hpp
/// \brief Defines utilities and a ROS 2 node for setting an entity's pose in a Gazebo simulation.

#ifndef ROS_GZ_SIM__SET_ENTITY_POSE_HPP_
#define ROS_GZ_SIM__SET_ENTITY_POSE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

/// \brief A ROS 2 node for setting entity poses in a Gazebo simulation.
class EntityPoseSetter : public rclcpp::Node {
public:
  /// \brief Default constructor. Initializes the node and pose setter client.
  EntityPoseSetter();

  /// \brief Constructor that accepts an external pose service client.
  /// \param[in] client Shared pointer to the set entity pose service client.
  explicit EntityPoseSetter(
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client);

  /// \brief Sets the pose of an entity in the simulation.
  /// \param[in] entity_name Name of the entity.
  /// \param[in] entity_id ID of the entity.
  /// \param[in] entity_type Type of the entity.
  /// \param[in] x X position coordinate.
  /// \param[in] y Y position coordinate.
  /// \param[in] z Z position coordinate.
  /// \param[in] qx First orientation parameter (quaternion X component or roll angle).
  /// \param[in] qy Second orientation parameter (quaternion Y component or pitch angle).
  /// \param[in] qz Third orientation parameter (quaternion Z component or yaw angle).
  /// \param[in] qw Fourth orientation parameter (quaternion W component, ignored for ]
  ///            Euler angles).
  /// \param[in] use_quaternion If true, orientation parameters are interpreted as quaternion
  ///            components. If false, they are interpreted as Euler angles (roll, pitch, yaw).
  /// \return True if the entity pose was set successfully, false otherwise.
  bool set_entity_pose(
    const std::string & entity_name, int entity_id,
    int entity_type, double x, double y, double z, double qx,
    double qy, double qz, double qw,
    bool use_quaternion = true);

protected:
  /// \brief Client used to call the set entity pose service.
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
};

#endif  // ROS_GZ_SIM__SET_ENTITY_POSE_HPP_
