// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef WORLD_NAME_HPP_
#define WORLD_NAME_HPP_

#include <gz/msgs/stringmsg_v.pb.h>

#include <optional>
#include <string>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

/// \brief Get the name of the first world running in the Gazebo server.
/// \param[in] ros2_node Node used for logging.
/// \return The world name, or std::nullopt if it couldn't be retrieved.
inline std::optional<std::string> retrieve_world_name(const rclcpp::Node::SharedPtr & ros2_node)
{
  gz::transport::Node node;
  bool executed{false};
  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/gazebo/worlds"};
  gz::msgs::StringMsg_V worlds_msg;

  // This loop is here to allow the server time to download resources.
  while (rclcpp::ok() && !executed) {
    RCLCPP_INFO(ros2_node->get_logger(), "Requesting list of world names.");
    executed = node.Request(service, timeout, worlds_msg, result);
  }

  if (!executed) {
    RCLCPP_INFO(ros2_node->get_logger(), "Timed out when getting world names.");
    return std::nullopt;
  }

  if (!result || worlds_msg.data().empty()) {
    RCLCPP_INFO(ros2_node->get_logger(), "Failed to get world names.");
    return std::nullopt;
  }

  return worlds_msg.data(0);
}

#endif  // WORLD_NAME_HPP_
