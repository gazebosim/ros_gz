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

/// \brief Enumeration for bridge warning types
enum class BridgeWarningType
{
  /// \brief No warning
  NONE,

  /// \brief Gazebo topic has multiple types or no types
  GZ_TYPE_UNDETERMINED,

  /// \brief No mapping found from current Gazebo type to ROS type
  GZ_TO_ROS_MAPPING_NOT_FOUND,

  /// \brief Failed to discover ROS topic info
  ROS_TYPE_DISCOVERED_FAILED,

  /// \brief ROS topic has multiple types or no types
  ROS_TYPE_UNDETERMINED,

  /// \brief Mismatch between the detected Gazebo and ROS types.
  ROS_GZ_TYPE_MISMATCH,
};

/// Forward declarations
class BridgeHandle;
class ServiceFactoryInterface;

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
  /// \param[in] ros_type_name Name of the ROS service (eg ros_gz_interfaces/srv/ControlWorld)
  /// \param[in] gz_req_type_name Gazebo service request type
  /// \param[in] gz_rep_type_name Gazebo service response type
  /// \param[in] service_name Address of the service to be bridged
  /// \param[in] factory Factory to create the service bridge
  void add_service_bridge(
    const std::string & ros_type_name,
    const std::string & gz_req_type_name,
    const std::string & gz_rep_type_name,
    const std::string & service_name,
    std::shared_ptr<ServiceFactoryInterface> factory = nullptr);

  void create_automated_bridges();

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

  /// \brief Get Gazebo topic information, including type and direction
  /// \param[in] topic_name Name of the Gazebo topic
  /// \param[out] gz_type_name Type of the Gazebo topic
  /// \param[out] direction Direction of the bridge
  /// \return True if successfully gets the Gazebo topic information
  bool get_gz_topic_info(const std::string & topic_name,
    std::string & gz_type_name, BridgeDirection & direction);
  
  /// \brief Get ROS topic information, including type
  /// \param[in] topic_name Name of the ROS topic
  /// \param[out] ros_type_name Type of the ROS topic
  /// \param[in] direction Direction of the bridge
  /// \return True if successfully gets the ROS topic information
  bool get_ros_topic_info(const std::string & topic_name,
    std::string & ros_type_name, const BridgeDirection & direction);

  /// \brief Get Gazebo service information, including request and response types
  /// \param[in] service_name Name of the Gazebo service
  /// \param[out] gz_req_type_name Type of the Gazebo service request
  /// \param[out] gz_rep_type_name Type of the Gazebo service response
  /// \return True if successfully gets the Gazebo service information
  bool get_gz_service_info(const std::string & service_name,
    std::string & gz_req_type_name, std::string & gz_rep_type_name);

  /// \brief Get ROS service information, including type
  /// \param[in] ros_services Map of ROS services and their types
  /// \param[in] service_name Name of the ROS service
  /// \param[out] ros_type_name Type of the ROS service
  /// \return True if successfully gets the ROS service information
  bool get_ros_service_info(
    const std::map<std::string, std::vector<std::string>> & ros_services,
    const std::string & service_name, std::string & ros_type_name);

  /// \brief Log a bridge warning while avoiding repeated messages for the same
  /// topic /service and warning type
  /// \param[in] warning_type Type of warning to log
  /// \param[in] name Topic /service associated with the warning.
  /// \param[in] resource_type Type of resource (topic or service)
  /// \param[in] ros_type_name ROS message type related to the warning
  /// \param[in] gz_type_name Gazebo message type related to the warning
  /// \param[in] extra_info Additional warning context
  void log_bridge_warning(
    const BridgeWarningType & warning_type,
    const std::string & name,
    const std::string & resource_type = "topic",
    const std::string & ros_type_name = "",
    const std::string & gz_type_name = "",
    const std::string & extra_info = "");

protected:
  /// \brief Pointer to Gazebo node used to create publishers/subscribers
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<ros_gz_bridge::BridgeHandle>> handles_;

  /// \brief List of bridged ROS services
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  /// \brief Map of bridge warnings
  std::map<std::string, BridgeWarningType> bridge_warnings_;
};
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
