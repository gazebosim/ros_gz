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

#ifndef ROS1_IGN_BRIDGE__FACTORY_HPP_
#define ROS1_IGN_BRIDGE__FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include <ignition/transport/Node.hh>

// include ROS 1 message event
#include <ros/message.h>
#include <ros/ros.h>

#include "ros1_ign_bridge/factory_interface.hpp"

namespace ros1_ign_bridge
{

template<typename ROS1_T, typename IGN_T>
class Factory : public FactoryInterface
{
public:
  Factory(
    const std::string & ros1_type_name, const std::string & ign_type_name)
  : ros1_type_name_(ros1_type_name),
    ign_type_name_(ign_type_name)
  {}

  ros::Publisher
  create_ros1_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size)
  {
    return node.advertise<ROS1_T>(topic_name, queue_size);
  }

  ignition::transport::Node::Publisher
  create_ign_publisher(
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & topic_name,
    size_t /*queue_size*/)
  {
    return ign_node->Advertise<IGN_T>(topic_name);
  }

  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    ignition::transport::Node::Publisher & ign_pub)
  {
    // workaround for https://github.com/ros/roscpp_core/issues/22 to get the
    // connection header
    ros::SubscribeOptions ops;
    ops.topic = topic_name;
    ops.queue_size = queue_size;
    ops.md5sum = ros::message_traits::md5sum<ROS1_T>();
    ops.datatype = ros::message_traits::datatype<ROS1_T>();
    ops.helper = ros::SubscriptionCallbackHelperPtr(
      new ros::SubscriptionCallbackHelperT
        <const ros::MessageEvent<ROS1_T const> &>(
          boost::bind(
            &Factory<ROS1_T, IGN_T>::ros1_callback,
            _1, ign_pub, ros1_type_name_, ign_type_name_)));
    return node.subscribe(ops);
  }

  void
  create_ign_subscriber(
    std::shared_ptr<ignition::transport::Node> node,
    const std::string & topic_name,
    size_t /*queue_size*/,
    ros::Publisher ros1_pub)
  {

    std::function<void(const IGN_T&,
                       const ignition::transport::MessageInfo &)> subCb =
    [this, ros1_pub](const IGN_T &_msg,
                     const ignition::transport::MessageInfo &_info)
    {
      // Ignore messages that are published from this bridge.
      if (!_info.IntraProcess())
        this->ign_callback(_msg, ros1_pub);
    };

    node->Subscribe(topic_name, subCb);
  }

protected:

  static
  void ros1_callback(
    const ros::MessageEvent<ROS1_T const> & ros1_msg_event,
    ignition::transport::Node::Publisher & ign_pub,
    const std::string & /*ros1_type_name*/,
    const std::string & /*ign_type_name*/)
  {
    const boost::shared_ptr<ros::M_string> & connection_header =
      ros1_msg_event.getConnectionHeaderPtr();
    if (!connection_header) {
      std::cerr << "  dropping message without connection header" << std::endl;
      return;
    }

    std::string key = "callerid";
    if (connection_header->find(key) != connection_header->end()) {
      if (connection_header->at(key) == ros::this_node::getName()) {
        return;
      }
    }

    const boost::shared_ptr<ROS1_T const> & ros1_msg =
      ros1_msg_event.getConstMessage();

    IGN_T ign_msg;
    convert_1_to_ign(*ros1_msg, ign_msg);
    ign_pub.Publish(ign_msg);
  }

  static
  void ign_callback(
    const IGN_T & ign_msg,
    ros::Publisher ros1_pub)
  {
    ROS1_T ros1_msg;
    convert_ign_to_1(ign_msg, ros1_msg);
    ros1_pub.publish(ros1_msg);
  }

public:
  // since convert functions call each other for sub messages they must be
  // public defined outside of the class
  static
  void
  convert_1_to_ign(
    const ROS1_T & ros1_msg,
    IGN_T & ign_msg);
  static
  void
  convert_ign_to_1(
    const IGN_T & ign_msg,
    ROS1_T & ros1_msg);

  std::string ros1_type_name_;
  std::string ign_type_name_;
};

}  // namespace ros1_ign_bridge

#endif  // ROS1_BRIDGE__FACTORY_HPP_
