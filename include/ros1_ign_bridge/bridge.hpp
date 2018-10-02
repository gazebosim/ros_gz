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
#include "ros1_ign_bridge/factory.hpp"
#include "ros1_ign_bridge/factory_interface.hpp"

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

// std::shared_ptr<FactoryInterface>
// get_factory(
//   const std::string & ros1_type_name,
//   const std::string & ros2_type_name);

Bridge1toIgnHandles
create_bridge_from_ros_to_ign(
  ros::NodeHandle /*ros1_node*/,
  std::shared_ptr<ignition::transport::Node> /*ign_node*/,
  const std::string & /*ros1_type_name*/,
  const std::string & /*ros1_topic_name*/,
  size_t /*subscriber_queue_size*/,
  const std::string & /*ign_type_name*/,
  const std::string & /*ign_topic_name*/,
  size_t /*publisher_queue_size*/)
{
  //auto factory = get_factory(ros1_type_name, ign_type_name);
  //auto ign_pub = factory->create_ign_publisher(
  //  ign_node, ign_topic_name, publisher_queue_size);

  //auto ros1_sub = factory->create_ros1_subscriber(
  //  ros1_node, ros1_topic_name, subscriber_queue_size, ign_pub);

  Bridge1toIgnHandles handles;
  // handles.ros1_subscriber = ros1_sub;
  // handles.ign_publisher = ign_pub;
  return handles;
}

}  // namespace ros1_ign_bridge

#endif  // ROS1_BRIDGE__BRIDGE_HPP_
