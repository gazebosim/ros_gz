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

#ifndef  ROS_IGN_BRIDGE__FACTORY_INTERFACE_HPP_
#define  ROS_IGN_BRIDGE__FACTORY_INTERFACE_HPP_

#include <string>

// include ROS
#include <ros/node_handle.h>
#include <ros/publisher.h>

// include Ignition Transport
#include <ignition/transport/Node.hh>

namespace ros_ign_bridge
{

class FactoryInterface
{
public:
  virtual
  ros::Publisher
  create_ros_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  ignition::transport::Node::Publisher
  create_ign_publisher(
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  ros::Subscriber
  create_ros_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    ignition::transport::Node::Publisher & ign_pub) = 0;

  virtual
  void
  create_ign_subscriber(
    std::shared_ptr<ignition::transport::Node> node,
    const std::string & topic_name,
    size_t queue_size,
    ros::Publisher ros_pub) = 0;
};

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__FACTORY_INTERFACE_HPP_
