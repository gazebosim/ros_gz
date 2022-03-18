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

#include <ignition/transport/Node.hh>
#include <rclcpp/node.hpp>

#include <memory>
#include <string>

#include "factory_interface.hpp"

namespace ros_ign_bridge
{

/// \brief Core functionality and data for both bridge directions
class Bridge
{
public:
  /// \brief Default subscriber queue length
  static constexpr size_t kDefaultSubscriberQueue = 10;

  /// \brief Default publisher queue length
  static constexpr size_t kDefaultPublisherQueue = 10;

  /// \brief Constructor
  /// Note, does not actually start the bridge, must be done with Start()
  ///
  /// \param[in] ros_node ROS node to create publishers/subscribers on
  /// \param[in] ign_node IGN node to create publishers/subscribers on
  /// \param[in] ros_type_name Name of the ROS message type to use
  /// \param[in] ros_topic Name of the ROS topic to use
  /// \param[in] ign_type_name Name of the IGN message type to use
  /// \param[in] ign_topic Name of the IGN topic to use
  /// \param[in] subscriber_queue_size Depth of the subscriber queue
  /// \param[in] publisher_queue_size Depth of the publisher queue
  /// \param[in] is_lazy True for "lazy" subscriptions.
  Bridge(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & ros_type_name,
    const std::string & ros_topic_name,
    const std::string & ign_type_name,
    const std::string & ign_topic_name,
    size_t subscriber_queue_size = kDefaultSubscriberQueue,
    size_t publisher_queue_size = kDefaultPublisherQueue,
    bool is_lazy = false);

  /// \brief Destructor
  virtual ~Bridge() = 0;

  /// \brief Initiate the bridge
  ///
  /// If the bridge is "lazy", then only the publisher (output) side is created.
  /// The not, both the publisher (output) and subscriber (input) side are
  /// created.
  void Start();

  /// \brief Spin the bridge, checking for new subscriptions on the output.
  ///
  /// Not necessary if the Bridge isn't lazy
  void Spin();

  /// \brief Inidicate if this is a "lazy" bridge
  ///
  /// A lazy bridge will always create the output side of the bridge, but
  /// will only create the input side of the bridge when downstream consumers
  /// are detected on the output side.
  /// This reduces the amount of overhead in the bridge processing messages
  /// for non-existent consumers.
  /// It will also allow for lazy publishers to work on the input side of the
  /// bridge.
  /// \return true if lazy
  bool IsLazy() const;

protected:
  /// \brief Get the number of subscriptions on the output side of the bridge
  virtual size_t NumSubscriptions() const = 0;

  /// \brief Indicate if the publisher (output) side has been created
  virtual bool HasPublisher() const = 0;

  /// \brief Start the publisher (output) side of the bridge
  virtual void StartPublisher() = 0;

  /// \brief Indicate if the subscriber (input) side has been created
  virtual bool HasSubscriber() const = 0;

  /// \brief Start the subscriber (input) side of the bridge
  virtual void StartSubscriber() = 0;

  /// \brief Stop the subscriber (input) side of the bridge
  virtual void StopSubscriber() = 0;

protected:
  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;

  std::string ros_type_name_;
  std::string ros_topic_name_;
  std::string ign_type_name_;
  std::string ign_topic_name_;

  size_t subscriber_queue_size_;
  size_t publisher_queue_size_;
  bool is_lazy_;

  std::shared_ptr<FactoryInterface> factory_;
};

using BridgePtr = std::unique_ptr<Bridge>;

}  // namespace ros_ign_bridge

#endif  // BRIDGE_HPP_
