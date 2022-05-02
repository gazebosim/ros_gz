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

#ifndef ROS_IGN_BRIDGE__ROS_IGN_BRIDGE_HPP_
#define ROS_IGN_BRIDGE__ROS_IGN_BRIDGE_HPP_

#include <ignition/msgs/config.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/node.hpp>

#include <memory>
#include <string>
#include <vector>

#include "ros_ign_bridge/bridge_config.hpp"

// Dataframe is available from versions 8.4.0 (fortress) forward
// This can be removed when the minimum supported version passes 8.4.0
#if (IGNITION_MSGS_MAJOR_VERSION >= 8 && \
  IGNITION_MSGS_MINOR_VERSION >= 4 && \
  IGNITION_MSGS_PATCH_VERSION >= 0)
#define HAVE_DATAFRAME true
#endif

namespace ros_ign_bridge
{
/// Forward declarations
class BridgeHandle;

/// \brief Component container for the ROS-IGN Bridge
class RosIgnBridge : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] options options control creation of the ROS 2 node
  explicit RosIgnBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// \brief Add a new ROS-IGN bridge to the node
  /// \param[in] config Parameters to control creation of a new bridge
  void add_bridge(const BridgeConfig & config);

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

protected:
  /// \brief Pointer to Ignition node used to create publishers/subscribers
  std::shared_ptr<ignition::transport::Node> ign_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<ros_ign_bridge::BridgeHandle>> handles_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__ROS_IGN_BRIDGE_HPP_
