// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef BRIDGE_HPP_
#define BRIDGE_HPP_

#include <memory>
#include <string>

#include <ignition/transport/Node.hh>

#include "factories.hpp"

namespace ros_ign_bridge
{

struct BridgeRosToIgnHandles
{
  rclcpp::SubscriptionBase::SharedPtr ros_subscriber;
  ignition::transport::Node::Publisher ign_publisher;
};

struct BridgeIgnToRosHandles
{
  std::shared_ptr<ignition::transport::Node> ign_subscriber;
  rclcpp::PublisherBase::SharedPtr ros_publisher;
};

struct BridgeHandles
{
  BridgeRosToIgnHandles bridgeRosToIgn;
  BridgeIgnToRosHandles bridgeIgnToRos;
};

struct BridgeIgnServicesToRosHandles
{
  std::shared_ptr<ignition::transport::Node> ign_node;
  rclcpp::ServiceBase::SharedPtr ros_service;
};

BridgeRosToIgnHandles
create_bridge_from_ros_to_ign(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  size_t subscriber_queue_size,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  size_t publisher_queue_size)
{
  auto factory = get_factory(ros_type_name, ign_type_name);
  auto ign_pub = factory->create_ign_publisher(ign_node, ign_topic_name, publisher_queue_size);

  auto ros_sub = factory->create_ros_subscriber(
    ros_node, ros_topic_name, subscriber_queue_size, ign_pub);

  BridgeRosToIgnHandles handles;
  handles.ros_subscriber = ros_sub;
  handles.ign_publisher = ign_pub;
  return handles;
}

BridgeIgnToRosHandles
create_bridge_from_ign_to_ros(
  std::shared_ptr<ignition::transport::Node> ign_node,
  rclcpp::Node::SharedPtr ros_node,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  size_t subscriber_queue_size,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  size_t publisher_queue_size)
{
  auto factory = get_factory(ros_type_name, ign_type_name);
  auto ros_pub = factory->create_ros_publisher(ros_node, ros_topic_name, publisher_queue_size);

  factory->create_ign_subscriber(ign_node, ign_topic_name, subscriber_queue_size, ros_pub);

  BridgeIgnToRosHandles handles;
  handles.ign_subscriber = ign_node;
  handles.ros_publisher = ros_pub;
  return handles;
}

BridgeHandles
create_bidirectional_bridge(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros_type_name,
  const std::string & ign_type_name,
  const std::string & topic_name,
  size_t queue_size = 10)
{
  BridgeHandles handles;
  handles.bridgeRosToIgn = create_bridge_from_ros_to_ign(
    ros_node, ign_node,
    ros_type_name, topic_name, queue_size, ign_type_name, topic_name, queue_size);
  handles.bridgeIgnToRos = create_bridge_from_ign_to_ros(
    ign_node, ros_node,
    ign_type_name, topic_name, queue_size, ros_type_name, topic_name, queue_size);
  return handles;
}

BridgeIgnServicesToRosHandles
create_service_bridge(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros_type_name,
  const std::string & ign_req_type_name,
  const std::string & ign_rep_type_name,
  const std::string & service_name)
{
  BridgeIgnServicesToRosHandles handles;
  auto factory = get_service_factory(ros_type_name, ign_req_type_name, ign_rep_type_name);
  auto ros_srv = factory->create_ros_service(ros_node, ign_node, service_name);
  handles.ros_service = ros_srv;
  handles.ign_node = ign_node;
  return handles;
}

}  // namespace ros_ign_bridge

#endif  // BRIDGE_HPP_
