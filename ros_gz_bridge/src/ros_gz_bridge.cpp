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

#include <memory>
#include <string>

#include <resource_retriever/retriever.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include "bridge_handle_ros_to_gz.hpp"
#include "bridge_handle_gz_to_ros.hpp"

namespace ros_gz_bridge
{

RosGzBridge::RosGzBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros_gz_bridge", options),
  config_file_parsed_(false)
{
  gz_node_ = std::make_shared<gz::transport::Node>();

  this->declare_parameter<int>("subscription_heartbeat", 1000);
  this->declare_parameter<std::string>("config_file", "");

  int heartbeat;
  this->get_parameter("subscription_heartbeat", heartbeat);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat),
    std::bind(&RosGzBridge::spin, this));
}

void RosGzBridge::spin()
{
  if (handles_.empty()) {
    std::string config_file;
    this->get_parameter("config_file", config_file);
    if (!config_file.empty() && !config_file_parsed_) {
      config_file_parsed_ = true;
      resource_retriever::Retriever r;
      try {
        resource_retriever::MemoryResource res = r.get(config_file);
        std::string str(reinterpret_cast<char *>(res.data.get()), res.size);
        auto entries = readFromYamlString(str);
        for (const auto & entry : entries) {
          this->add_bridge(entry);
        }
      } catch (const resource_retriever::Exception & exc) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unable to find: [%s]", config_file.c_str());
      }
    }
  }

  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}

void RosGzBridge::add_bridge(const BridgeConfig & config)
{
  bool gz_to_ros = false;
  bool ros_to_gz = false;

  if (config.direction == BridgeDirection::GZ_TO_ROS) {
    gz_to_ros = true;
  }

  if (config.direction == BridgeDirection::ROS_TO_GZ) {
    ros_to_gz = true;
  }

  if (config.direction == BridgeDirection::BIDIRECTIONAL) {
    ros_to_gz = true;
    gz_to_ros = true;
  }

  try {
    if (gz_to_ros) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating GZ->ROS Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.gz_topic_name.c_str(), config.gz_type_name.c_str(),
        config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
        config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleGzToRos>(
          shared_from_this(), gz_node_,
          config));

      handles_.back()->Start();
    }

    if (ros_to_gz) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating ROS->GZ Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
        config.gz_topic_name.c_str(), config.gz_type_name.c_str(),
        config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleRosToGz>(
          shared_from_this(), gz_node_,
          config));

      handles_.back()->Start();
    }
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
      "to topic [%s] with Gazebo Transport type [%s]",
      config.ros_topic_name.c_str(),
      config.ros_type_name.c_str(),
      config.gz_topic_name.c_str(),
      config.gz_type_name.c_str());
  }
}

void RosGzBridge::add_service_bridge(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name,
  const std::string & service_name)
{
  auto factory = get_service_factory(ros_type_name, gz_req_type_name, gz_rep_type_name);
  services_.push_back(factory->create_ros_service(shared_from_this(), gz_node_, service_name));
}

}  // namespace ros_gz_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_bridge::RosGzBridge)
