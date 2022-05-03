// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <ros_ign_bridge/ros_ign_bridge.hpp>

#include <memory>
#include <string>

#include "bridge_handle_ros_to_ign.hpp"
#include "bridge_handle_ign_to_ros.hpp"

namespace ros_ign_bridge
{

RosIgnBridge::RosIgnBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros_ign_bridge", options)
{
  ign_node_ = std::make_shared<ignition::transport::Node>();

  this->declare_parameter<int>("subscription_heartbeat", 1000);
  this->declare_parameter<std::string>("config_file", "");

  int heartbeat;
  this->get_parameter("subscription_heartbeat", heartbeat);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat),
    std::bind(&RosIgnBridge::spin, this));
}

void RosIgnBridge::spin()
{
  if (handles_.empty()) {
    std::string config_file;
    this->get_parameter("config_file", config_file);
    if (!config_file.empty()) {
      auto entries = readFromYamlFile(config_file);
      for (const auto & entry : entries) {
        this->add_bridge(entry);
      }
    }
  }

  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}

void RosIgnBridge::add_bridge(const BridgeConfig & config)
{
  bool ign_to_ros = false;
  bool ros_to_ign = false;

  if (config.direction == BridgeDirection::IGN_TO_ROS) {
    ign_to_ros = true;
  }

  if (config.direction == BridgeDirection::ROS_TO_IGN) {
    ros_to_ign = true;
  }

  if (config.direction == BridgeDirection::BIDIRECTIONAL) {
    ros_to_ign = true;
    ign_to_ros = true;
  }

  try {
    if (ign_to_ros) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating IGN->ROS Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.ign_topic_name.c_str(), config.ign_type_name.c_str(),
        config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
        config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_ign_bridge::BridgeHandleIgnToRos>(
          shared_from_this(), ign_node_,
          config));

      handles_.back()->Start();
    }

    if (ros_to_ign) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating ROS->IGN Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
        config.ign_topic_name.c_str(), config.ign_type_name.c_str(),
        config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_ign_bridge::BridgeHandleRosToIgn>(
          shared_from_this(), ign_node_,
          config));

      handles_.back()->Start();
    }
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
      "to topic [%s] with Ignition Transport type [%s]",
      config.ros_topic_name.c_str(),
      config.ign_topic_name.c_str(),
      config.ros_type_name.c_str(),
      config.ign_type_name.c_str());
  }
}
}  // namespace ros_ign_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_ign_bridge::RosIgnBridge)
