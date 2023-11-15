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

#ifndef BRIDGE_HANDLE_ROS_TO_GZ_HPP_
#define BRIDGE_HANDLE_ROS_TO_GZ_HPP_

#include <gz/transport/Node.hh>
#include <rclcpp/subscription_base.hpp>

#include "bridge_handle.hpp"

namespace ros_gz_bridge
{

/// \brief Create a bridge ROS to GZ Bridge
///
/// Bridge is from ROS Subscription (input) to GZ Publisher (output)
class BridgeHandleRosToGz : public BridgeHandle
{
public:
  /// \brief Constructor
  using BridgeHandle::BridgeHandle;

  /// \brief Destructor
  ~BridgeHandleRosToGz() override;

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

  /// \brief Gazebo publisher, populated when publisher active
  gz::transport::Node::Publisher gz_publisher_;
};

}  // namespace ros_gz_bridge

#endif  // BRIDGE_HANDLE_ROS_TO_GZ_HPP_
