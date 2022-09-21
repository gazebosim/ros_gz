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

#ifndef BRIDGE_HANDLE_ROS_TO_IGN_HPP_
#define BRIDGE_HANDLE_ROS_TO_IGN_HPP_

#include <ignition/transport/Node.hh>
#include <rclcpp/subscription_base.hpp>

#include "bridge_handle.hpp"

namespace ros_ign_bridge
{

/// \brief Create a bridge ROS to IGN Bridge
///
/// Bridge is from ROS Subscription (input) to IGN Publisher (output)
class BridgeHandleRosToIgn : public BridgeHandle
{
public:
  /// \brief Constructor
  using BridgeHandle::BridgeHandle;

  /// \brief Destructor
  ~BridgeHandleRosToIgn() override;

protected:
  /// \brief Documentation inherited
  size_t NumSubscriptions() const override;

  /// \brief Documentation inherited
  bool HasPublisher() const override;

  /// \brief Documentation inherited
  void StartPublisher() override;

  /// \brief Documentation inherited
  bool HasSubscriber() const override;

  /// \brief Documentation inherited
  void StartSubscriber() override;

  /// \brief Documentation inherited
  void StopSubscriber() override;

protected:
  /// \brief ROS subscriber, populated when subscription active
  rclcpp::SubscriptionBase::SharedPtr ros_subscriber_ = {nullptr};

  /// \brief Ignition publisher, populated when publisher active
  ignition::transport::Node::Publisher ign_publisher_;
};

}  // namespace ros_ign_bridge

#endif  // BRIDGE_HANDLE_ROS_TO_IGN_HPP_
