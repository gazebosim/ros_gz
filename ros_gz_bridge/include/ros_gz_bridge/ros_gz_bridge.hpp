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

#ifndef ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
#define ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/config.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>
#include "ros_gz_bridge/bridge_config.hpp"

namespace ros_gz_bridge
{
/// Forward declarations
class BridgeHandle;

/// \brief Component container for the ROS-GZ Bridge
class RosGzBridge : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] options options control creation of the ROS 2 node
  explicit RosGzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// \brief Add a new ROS-GZ bridge to the node
  /// \param[in] config Parameters to control creation of a new bridge
  void add_bridge(const BridgeConfig & config);

  /// \brief Create a new ROS-GZ bridge for a service
  /// \param[in] ros_type_name Name of the ROS service (eg ros_bz_interfaces/srv/ControlWorld)
  /// \param[in] gz_req_type_name Gazebo service request type
  /// \param[in] gz_req_type_name Gazebo service response type
  /// \param[in] service_name Address of the service to be bridged
  void add_service_bridge(
    const std::string & ros_type_name,
    const std::string & gz_req_type_name,
    const std::string & gz_rep_type_name,
    const std::string & service_name);

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

protected:
  /// \brief Pointer to Gazebo node used to create publishers/subscribers
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<ros_gz_bridge::BridgeHandle>> handles_;

  /// \brief List of bridged ROS services
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
