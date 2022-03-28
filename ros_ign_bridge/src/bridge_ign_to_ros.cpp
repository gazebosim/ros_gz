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
//
#include <memory>

#include "bridge_ign_to_ros.hpp"

namespace ros_ign_bridge
{

BridgeIgnToRos::~BridgeIgnToRos() = default;

size_t BridgeIgnToRos::NumSubscriptions() const
{
  // Return number of ROS subscriptions
  size_t valid_subscriptions = 0;

  if (this->ros_publisher_ != nullptr) {
    // Use info_by_topic rather than get_subscription_count
    // to filter out potential bidirectional bridge
    auto topic_info = this->ros_node_->get_subscriptions_info_by_topic(
      this->ros_topic_name_);

    for (auto & topic : topic_info) {
      if (topic.node_name() == this->ros_node_->get_name()) {
        continue;
      }
      valid_subscriptions++;
    }
  }

  return valid_subscriptions;
}

bool BridgeIgnToRos::HasPublisher() const
{
  return this->ros_publisher_ != nullptr;
}

void BridgeIgnToRos::StartPublisher()
{
  // Start ROS publisher
  this->ros_publisher_ = this->factory_->create_ros_publisher(
    this->ros_node_,
    this->ros_topic_name_,
    this->publisher_queue_size_);
}

bool BridgeIgnToRos::HasSubscriber() const
{
  // Return Ignition subscriber status
  return this->ign_subscriber_ != nullptr;
}

void BridgeIgnToRos::StartSubscriber()
{
  // Start Ignition subscriber
  this->factory_->create_ign_subscriber(
    this->ign_node_,
    this->ign_topic_name_,
    this->subscriber_queue_size_,
    this->ros_publisher_);

  this->ign_subscriber_ = this->ign_node_;
}

void BridgeIgnToRos::StopSubscriber()
{
  // Stop Ignition subscriber
  if (!this->ign_subscriber_) {
    return;
  }

  this->ign_subscriber_->Unsubscribe(this->ign_topic_name_);
  this->ign_subscriber_.reset();
}

}  // namespace ros_ign_bridge
