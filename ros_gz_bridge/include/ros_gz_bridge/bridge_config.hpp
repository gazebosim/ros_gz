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
//
#ifndef ROS_GZ_BRIDGE__BRIDGE_CONFIG_HPP_
#define ROS_GZ_BRIDGE__BRIDGE_CONFIG_HPP_

#include <string>
#include <vector>

namespace ros_gz_bridge
{

/// \brief Enumeration for bridge direction
enum class BridgeDirection
{
  NONE = 0,
  BIDIRECTIONAL = 1,
  GZ_TO_ROS = 2,
  ROS_TO_GZ = 3,
};

/// \brief Default subscriber queue length
static constexpr size_t kDefaultSubscriberQueue = 10;

/// \brief Default publisher queue length
static constexpr size_t kDefaultPublisherQueue = 10;

// \brief Default lazy param
static constexpr bool kDefaultLazy = false;

// \brief Default bridge connectivity
static constexpr BridgeDirection kDefaultDirection = BridgeDirection::BIDIRECTIONAL;

struct BridgeConfig
{
  /// \brief The ROS message type (eg std_msgs/msg/String)
  std::string ros_type_name;

  /// \brief The ROS topic name to bridge
  std::string ros_topic_name;

  /// \brief The Gazebo message type (eg ignition.msgs.String)
  /// Used with topic bridges
  std::string gz_type_name;

  /// \brief The Gazebo topic name to bridge
  std::string gz_topic_name;

  /// \brief Connectivity of the bridge
  BridgeDirection direction = kDefaultDirection;

  /// \brief Depth of the subscriber queue
  size_t subscriber_queue_size = kDefaultSubscriberQueue;

  /// \brief Depth of the publisher queue
  size_t publisher_queue_size = kDefaultPublisherQueue;

  /// \brief Flag to change the "laziness" of the bridge
  bool is_lazy = kDefaultLazy;
};

/// \brief Generate a group of BridgeConfigs from a YAML String
/// \param[in] data string containing YAML of bridge configurations
/// \return Vector of bridge configurations
std::vector<BridgeConfig> readFromYamlString(const std::string & data);

/// \brief Generate a group of BridgeConfigs from a YAML File
/// \param[in] filename name of file containing YAML of bridge configurations
/// \return Vector of bridge configurations
std::vector<BridgeConfig> readFromYamlFile(const std::string & filename);

}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__BRIDGE_CONFIG_HPP_
