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

#ifndef ROS1_IGN_BRIDGE__BRIDGE_HPP_
#define ROS1_IGN_BRIDGE__BRIDGE_HPP_

#include <map>
#include <memory>
#include <string>

// include ROS 1
#include "ros/node_handle.h"

#include <ignition/transport/Node.hh>
#include "ros1_ign_bridge/builtin_interfaces_factories.hpp"

namespace ros1_ign_bridge
{

struct Bridge1toIgnHandles
{
  ros::Subscriber ros1_subscriber;
  std::shared_ptr<ignition::transport::Node::Publisher> ign_publisher;
};

struct BridgeIgnto1Handles
{
  std::shared_ptr<ignition::transport::Node> ign_subscriber;
  ros::Publisher ros1_publisher;
};

struct BridgeHandles
{
  Bridge1toIgnHandles bridge1toIgn;
  BridgeIgnto1Handles bridgeIgnto1;
};

BridgeIgnto1Handles
create_bridge_from_ign_to_ros(
  std::shared_ptr<ignition::transport::Node> ign_node,
  ros::NodeHandle ros1_node,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  size_t subscriber_queue_size,
  const std::string & ros1_type_name,
  const std::string & ros1_topic_name,
  size_t publisher_queue_size)
{
  auto factory = get_factory(ros1_type_name, ign_type_name);
  auto ros1_pub = factory->create_ros1_publisher(
    ros1_node, ros1_topic_name, publisher_queue_size);

  factory->create_ign_subscriber(
    ign_node, ign_topic_name, subscriber_queue_size, ros1_pub);

  BridgeIgnto1Handles handles;
  handles.ign_subscriber = ign_node;
  handles.ros1_publisher = ros1_pub;
  return handles;
}

}  // namespace ros1_ign_bridge

#endif  // ROS1_BRIDGE__BRIDGE_HPP_
