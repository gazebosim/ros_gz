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

#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "bridge_handle_ros_to_gz.hpp"
#include "bridge_handle_gz_to_ros.hpp"
#include "get_mappings.hpp"

#include <rclcpp/expand_topic_or_service_name.hpp>

namespace ros_gz_bridge
{

RosGzBridge::RosGzBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros_gz_bridge", options)
{
  gz_node_ = std::make_shared<gz::transport::Node>();

  this->declare_parameter<int>("subscription_heartbeat", 1000);
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<bool>("lazy", kDefaultLazy);
  this->declare_parameter<bool>("expand_gz_topic_names", false);
  this->declare_parameter<bool>("override_timestamps_with_wall_time", false);
  this->declare_parameter<std::string>("override_frame_id", "");
  this->declare_parameter("bridge_names", std::vector<std::string>());
  this->declare_parameter("enable_automated_bridge", false);
  const auto names = this->get_parameter("bridge_names").as_string_array();

  using rclcpp::PARAMETER_STRING;
  using rclcpp::PARAMETER_NOT_SET;

  for (const auto & name : names) {
    const auto prefix = "bridges." + name + ".";

    const auto ros_type_name = this->declare_parameter(prefix + "ros_type_name", PARAMETER_STRING);
    if (ros_type_name.get_type() == PARAMETER_NOT_SET) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Bridge %s does not set required parameter ros_type_name.", name.c_str());
      continue;
    }

    const auto ros_topic_name = this->declare_parameter(prefix + "ros_topic_name", "");
    const auto service_name = this->declare_parameter(prefix + "service_name", "");
    const auto is_topic = !ros_topic_name.empty();
    const auto is_service = !service_name.empty();

    if (is_topic == is_service) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Bridge %s needs to set exactly one of ros_topic_name or service_name.", name.c_str());
      continue;
    }

    if (is_topic) {
      const auto gz_topic_name = this->declare_parameter(prefix + "gz_topic_name",
        PARAMETER_STRING);
      if (gz_topic_name.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_topic_name.", name.c_str());
        continue;
      }
      const auto gz_type_name = this->declare_parameter(prefix + "gz_type_name", PARAMETER_STRING);
      if (gz_type_name.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_type_name.", name.c_str());
        continue;
      }
      this->declare_parameter(prefix + "direction", "BIDIRECTIONAL");
      // Queue sizes default to 10 if qos_profile is not set.
      // If it is defined, they are applied only if they are non-negative.
      this->declare_parameter(prefix + "publisher_queue", -1);
      this->declare_parameter(prefix + "subscriber_queue", -1);
      this->declare_parameter(prefix + "lazy", this->get_parameter("lazy").as_bool());
      this->declare_parameter(prefix + "qos_profile", "");
      this->declare_parameter(prefix + "frame_id", "");
    } else {
      const auto gz_req_type = this->declare_parameter(prefix + "gz_req_type_name",
        PARAMETER_STRING);
      if (gz_req_type.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_req_type_name.", name.c_str());
        continue;
      }
      const auto gz_rep_type = this->declare_parameter(prefix + "gz_rep_type_name",
        PARAMETER_STRING);
      if (gz_rep_type.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Bridge %s does not set required parameter gz_rep_type_name.", name.c_str());
        continue;
      }
    }
  }

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
    bool expand_names;
    this->get_parameter("expand_gz_topic_names", expand_names);
    const std::string ros_ns = this->get_namespace();
    const std::string ros_node_name = this->get_name();

    bool lazy;
    this->get_parameter("lazy", lazy);

    // Add bridges from config file
    if (!config_file.empty()) {
      auto entries = readFromYamlFile(config_file);
      for (auto & entry : entries) {
        if (expand_names) {
          entry.gz_topic_name = rclcpp::expand_topic_or_service_name(
            entry.gz_topic_name, ros_node_name, ros_ns, false);
        }
        entry.is_lazy = entry.is_lazy.value_or(lazy);
        if (entry.service_name.empty()) {
          this->add_bridge(entry);
        } else {
          this->add_service_bridge(
            entry.ros_type_name,
            entry.gz_req_type_name,
            entry.gz_rep_type_name,
            entry.service_name);
        }
      }
    }

    // Add bridges from parameters
    const auto names = this->get_parameter("bridge_names").as_string_array();
    for (const auto & name : names) {
      const auto prefix = "bridges." + name + ".";
      if (!this->get_parameter(prefix + "ros_topic_name").as_string().empty()) {
        const auto directionStr = this->get_parameter(prefix + "direction").as_string();
        BridgeDirection direction {BridgeDirection::NONE};
        if (directionStr == "NONE") {
          direction = BridgeDirection::NONE;
        } else if (directionStr == "BIDIRECTIONAL") {
          direction = BridgeDirection::BIDIRECTIONAL;
        } else if (directionStr == "GZ_TO_ROS") {
          direction = BridgeDirection::GZ_TO_ROS;
        } else if (directionStr == "ROS_TO_GZ") {
          direction = BridgeDirection::ROS_TO_GZ;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "Bridge %s defines unknown direction %s.",
            name.c_str(), directionStr.c_str());
          continue;
        }

        const auto qos_profile_str = this->get_parameter(prefix + "qos_profile").as_string();
        std::optional<rclcpp::QoS> qos_profile;
        if (!qos_profile_str.empty()) {
          try {
            qos_profile = parseQoS(qos_profile_str);
          } catch (const std::invalid_argument & e) {
            RCLCPP_ERROR(
              this->get_logger(),
              "Bridge %s defines unknown QoS profile %s.",
              name.c_str(), qos_profile_str.c_str());
            continue;
          }
        }

        const auto pub_queue_size_int = this->get_parameter(prefix + "publisher_queue").as_int();
        std::optional<size_t> pub_queue_size;
        if (pub_queue_size_int >= 0) {
          pub_queue_size = pub_queue_size_int;
        } else if (!qos_profile.has_value()) {
          pub_queue_size = kDefaultPublisherQueue;
        }

        const auto sub_queue_size_int = this->get_parameter(prefix + "subscriber_queue").as_int();
        std::optional<size_t> sub_queue_size;
        if (sub_queue_size_int >= 0) {
          sub_queue_size = sub_queue_size_int;
        } else if (!qos_profile.has_value()) {
          sub_queue_size = kDefaultSubscriberQueue;
        }

        BridgeConfig config {
          this->get_parameter(prefix + "ros_type_name").as_string(),
          this->get_parameter(prefix + "ros_topic_name").as_string(),
          this->get_parameter(prefix + "gz_type_name").as_string(),
          this->get_parameter(prefix + "gz_topic_name").as_string(),
          direction,
          sub_queue_size,
          pub_queue_size,
          this->get_parameter(prefix + "lazy").as_bool(),
          qos_profile,
          {},
          {},
          {},
          this->get_parameter(prefix + "frame_id").as_string()
        };
        if (expand_names) {
          config.gz_topic_name = rclcpp::expand_topic_or_service_name(
            config.gz_topic_name, ros_node_name, ros_ns, false);
        }

        this->add_bridge(config);
      } else {
        this->add_service_bridge(
          this->get_parameter(prefix + "ros_type_name").as_string(),
          this->get_parameter(prefix + "gz_req_type_name").as_string(),
          this->get_parameter(prefix + "gz_rep_type_name").as_string(),
          this->get_parameter(prefix + "service_name").as_string());
      }
    }
  }

  bool enable_automated_bridge = false;
  this->get_parameter("enable_automated_bridge", enable_automated_bridge);
  if (enable_automated_bridge) {
    create_automated_bridges();
  }

  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}

void RosGzBridge::add_bridge(const BridgeConfig & input_config)
{
  // Resolve the laziness: if the caller left is_lazy as nullopt, inherit the
  // node-level "lazy" parameter so that the effective value is always explicit.
  BridgeConfig config = input_config;
  if (!config.is_lazy.has_value()) {
    bool node_lazy = kDefaultLazy;
    this->get_parameter("lazy", node_lazy);
    config.is_lazy = node_lazy;
  }
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
        config.is_lazy.value_or(kDefaultLazy));
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
        config.is_lazy.value_or(kDefaultLazy));
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
      "to topic [%s] with Gazebo Transport type [%s]: %s",
      config.ros_topic_name.c_str(),
      config.ros_type_name.c_str(),
      config.gz_topic_name.c_str(),
      config.gz_type_name.c_str(),
      _e.what());
  }
}

void RosGzBridge::add_service_bridge(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name,
  const std::string & service_name)
{
  try {
    RCLCPP_INFO(
      this->get_logger(),
      "Creating ROS->GZ service bridge [%s (%s -> %s/%s)]",
      service_name.c_str(), ros_type_name.c_str(),
      gz_req_type_name.c_str(), gz_rep_type_name.c_str());
    auto factory = get_service_factory(ros_type_name, gz_req_type_name, gz_rep_type_name);
    services_.push_back(factory->create_ros_service(shared_from_this(), gz_node_, service_name));
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for service [%s] with ROS2 type [%s] "
      " and Gazebo types [%s/%s]: %s",
      service_name.c_str(), ros_type_name.c_str(),
      gz_req_type_name.c_str(), gz_rep_type_name.c_str(),
      _e.what());
  }
}

// TODO: Avoid flooding the log with repeated messages during retries.
void RosGzBridge::create_automated_bridges()
{
  std::vector<std::string> gz_topics;
  gz_node_->TopicList(gz_topics);

  for (const auto & gz_topic : gz_topics)
  {
    // Skip topics that are already bridged
    bool already_bridged = false;
    for (const auto & handle : handles_)
    {
      if (handle->GetConfig().gz_topic_name == gz_topic)
      {
        already_bridged = true;
        break;
      }
    }
    if (already_bridged) {
      continue;
    }

    std::vector<gz::transport::MessagePublisher> gz_publishers;
    std::vector<gz::transport::MessagePublisher> gz_subscribers;
    if (!gz_node_->TopicInfo(gz_topic, gz_publishers, gz_subscribers)) {
      continue;
    }

    std::unordered_set<std::string> gz_publisher_types;
    for (const auto & pub : gz_publishers)
    {
      gz_publisher_types.insert(pub.MsgTypeName());
    }
    std::unordered_set<std::string> gz_subscriber_types;
    for (const auto & sub : gz_subscribers)
    {
      gz_subscriber_types.insert(sub.MsgTypeName());
    }

    BridgeDirection direction {BridgeDirection::NONE};
    std::string gz_type_name;
    if (gz_publisher_types.size() > 1 || gz_subscriber_types.size() > 1)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping automated bridge for topic [%s] for multiple "
        "Gazebo message types.",
        gz_topic.c_str());
      continue;
    }
    else if (gz_publisher_types.size() == 1 && gz_subscriber_types.size() == 1 &&
             *gz_publisher_types.begin() == *gz_subscriber_types.begin())
    {
      gz_type_name = *gz_publisher_types.begin();
      direction = BridgeDirection::BIDIRECTIONAL;
    }
    else if (gz_publisher_types.size() == 1 && gz_subscriber_types.size() == 0)
    {
      gz_type_name = *gz_publisher_types.begin();
      direction = BridgeDirection::GZ_TO_ROS;
    }
    else if (gz_publisher_types.size() == 0 && gz_subscriber_types.size() == 1)
    {
      gz_type_name = *gz_subscriber_types.begin();
      direction = BridgeDirection::ROS_TO_GZ;
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping automated bridge for topic [%s] for no Gazebo message "
        "types discovered.", gz_topic.c_str());
      continue;
    }

    std::vector<std::string> ros_candidate_types;
    if (!get_gz_to_ros_mapping(gz_type_name, ros_candidate_types))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping automated bridge for topic [%s] for Gazebo message type "
        "[%s] with no known ROS message type mapping.",
        gz_topic.c_str(), gz_type_name.c_str());
      continue;
    }

    std::string ros_type_name;
    if (direction == BridgeDirection::ROS_TO_GZ)
    {
      try
      {
        auto publishers = this->get_publishers_info_by_topic(gz_topic);

        std::unordered_set<std::string> ros_publisher_types;
        for (const auto & pub : publishers)
        {
          ros_publisher_types.insert(pub.topic_type());
        }
        if (ros_publisher_types.size() == 1)
        {
          ros_type_name = *ros_publisher_types.begin();
        }
        else
        {
          continue;
        }
      }
      catch(const std::exception& e)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skipping automated bridge for topic [%s] for ROS message type "
          "discovery error: %s",
          gz_topic.c_str(), e.what());
        continue;
      }
      
    }
    else if (direction == BridgeDirection::GZ_TO_ROS)
    {
      try
      {
        auto subscribers = this->get_subscriptions_info_by_topic(gz_topic);

        std::unordered_set<std::string> ros_subscriber_types;
        for (const auto & sub : subscribers)
        {
          ros_subscriber_types.insert(sub.topic_type());
        }
        if (ros_subscriber_types.size() == 1)
        {
          ros_type_name = *ros_subscriber_types.begin();
        }
        else
        {
          continue;
        }
      }
      catch(const std::exception& e)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skipping automated bridge for topic [%s] for ROS message type "
          "discovery error: %s",
          gz_topic.c_str(), e.what());
        continue;
      }
    }
    else if (direction == BridgeDirection::BIDIRECTIONAL)
    {
      try
      {
        auto publishers = this->get_publishers_info_by_topic(gz_topic);
        auto subscribers = this->get_subscriptions_info_by_topic(gz_topic);

        std::unordered_set<std::string> ros_publisher_types;
        for (const auto & pub : publishers)
        {
          ros_publisher_types.insert(pub.topic_type());
        }
        std::unordered_set<std::string> ros_subscriber_types;
        for (const auto & sub : subscribers)
        {
          ros_subscriber_types.insert(sub.topic_type());
        }

        if ((ros_publisher_types.size() > 1 || ros_subscriber_types.size() > 1) ||
            (ros_publisher_types.size() == 0 && ros_subscriber_types.size() == 0))
        {
          continue;
        }
        else if (ros_publisher_types.size() == 1 && ros_subscriber_types.size() == 1)
        {
          if (*ros_publisher_types.begin() == *ros_subscriber_types.begin())
          {
            ros_type_name = *ros_publisher_types.begin();
          }
          else
          {
            continue;
          }
        }
        else
        {
          if (ros_publisher_types.size() == 1)
          {
            ros_type_name = *ros_publisher_types.begin();
          }
          else
          {
            ros_type_name = *ros_subscriber_types.begin();
          }
        }
      }
      catch(const std::exception& e)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skipping automated bridge for topic [%s] for ROS message type "
          "discovery error: %s",
          gz_topic.c_str(), e.what());
        continue;
      }
    }

    bool is_mapping_valid = false;
    for (const auto & candidate : ros_candidate_types)
    {
      if (candidate == ros_type_name)
      {
        is_mapping_valid = true;
        break;
      }
    }

    if (!is_mapping_valid)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping automated bridge for topic [%s] for ROS message type "
        "[%s] with no known mapping to Gazebo message type [%s].",
        gz_topic.c_str(), ros_type_name.c_str(), gz_type_name.c_str());
      continue;
    }

    BridgeConfig config;
    config.ros_type_name = ros_type_name;
    config.ros_topic_name = gz_topic;
    config.gz_type_name = gz_type_name;
    config.gz_topic_name = gz_topic;
    config.direction = direction;
    this->add_bridge(config);
  }
}
}  // namespace ros_gz_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_bridge::RosGzBridge)
