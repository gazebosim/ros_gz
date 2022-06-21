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

#ifndef FACTORY_HPP_
#define FACTORY_HPP_

#include <ignition/transport/Node.hh>

// include ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>

#include <functional>
#include <memory>
#include <string>

#include "factory_interface.hpp"

namespace ros_ign_bridge
{

template<typename ROS_T, typename IGN_T>
class Factory : public FactoryInterface
{
public:
  Factory(
    const std::string & ros_type_name, const std::string & ign_type_name)
  : ros_type_name_(ros_type_name),
    ign_type_name_(ign_type_name)
  {}

  virtual ~Factory() {}

  rclcpp::PublisherBase::SharedPtr
  create_ros_publisher(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size)
  {
    // Allow QoS overriding
    auto options = rclcpp::PublisherOptions();
    options.qos_overriding_options = rclcpp::QosOverridingOptions {
      {
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability
      },
    };

    std::shared_ptr<rclcpp::Publisher<ROS_T>> publisher =
      ros_node->create_publisher<ROS_T>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), options);
    return publisher;
  }

  ignition::transport::Node::Publisher
  create_ign_publisher(
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & topic_name,
    size_t /*queue_size*/)
  {
    return ign_node->Advertise<IGN_T>(topic_name);
  }

  rclcpp::SubscriptionBase::SharedPtr
  create_ros_subscriber(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size,
    ignition::transport::Node::Publisher & ign_pub)
  {
    std::function<void(std::shared_ptr<const ROS_T>)> fn = std::bind(
      &Factory<ROS_T, IGN_T>::ros_callback,
      std::placeholders::_1, ign_pub,
      ros_type_name_, ign_type_name_,
      ros_node);
    auto options = rclcpp::SubscriptionOptions();
    // Ignore messages that are published from this bridge.
    options.ignore_local_publications = true;
    // Allow QoS overriding
    options.qos_overriding_options =
      rclcpp::QosOverridingOptions::with_default_policies();
    std::shared_ptr<rclcpp::Subscription<ROS_T>> subscription =
      ros_node->create_subscription<ROS_T>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), fn, options);
    return subscription;
  }

  void
  create_ign_subscriber(
    std::shared_ptr<ignition::transport::Node> node,
    const std::string & topic_name,
    size_t /*queue_size*/,
    rclcpp::PublisherBase::SharedPtr ros_pub)
  {
    std::function<void(const IGN_T &,
      const ignition::transport::MessageInfo &)> subCb =
      [this, ros_pub](const IGN_T & _msg,
        const ignition::transport::MessageInfo & _info)
      {
        // Ignore messages that are published from this bridge.
        if (!_info.IntraProcess()) {
          this->ign_callback(_msg, ros_pub);
        }
      };

    node->Subscribe(topic_name, subCb);
  }

protected:
  static
  void ros_callback(
    std::shared_ptr<const ROS_T> ros_msg,
    ignition::transport::Node::Publisher & ign_pub,
    const std::string & ros_type_name,
    const std::string & ign_type_name,
    rclcpp::Node::SharedPtr ros_node)
  {
    IGN_T ign_msg;
    convert_ros_to_ign(*ros_msg, ign_msg);
    ign_pub.Publish(ign_msg);
    RCLCPP_INFO_ONCE(
      ros_node->get_logger(),
      "Passing message from ROS %s to Ignition %s (showing msg only once per type)",
      ros_type_name.c_str(), ign_type_name.c_str());
  }

  static
  void ign_callback(
    const IGN_T & ign_msg,
    rclcpp::PublisherBase::SharedPtr ros_pub)
  {
    ROS_T ros_msg;
    convert_ign_to_ros(ign_msg, ros_msg);
    std::shared_ptr<rclcpp::Publisher<ROS_T>> pub =
      std::dynamic_pointer_cast<rclcpp::Publisher<ROS_T>>(ros_pub);
    if (pub != nullptr) {
      pub->publish(ros_msg);
    }
  }

public:
  // since convert functions call each other for sub messages they must be
  // public defined outside of the class
  static
  void
  convert_ros_to_ign(
    const ROS_T & ros_msg,
    IGN_T & ign_msg);
  static
  void
  convert_ign_to_ros(
    const IGN_T & ign_msg,
    ROS_T & ros_msg);

  std::string ros_type_name_;
  std::string ign_type_name_;
};

}  // namespace ros_ign_bridge

#endif  // FACTORY_HPP_
