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

#include "bridge_ros_to_ign.hpp"

#include <ignition/transport/TopicUtils.hh>

namespace ros_ign_bridge
{

BridgeRosToIgn::~BridgeRosToIgn() = default;

size_t BridgeRosToIgn::NumSubscriptions() const
{
  // Return number of ignition subscriptions
  // Ignition publishers can only detect presence of connections, and not
  // get the actual count.
  // Ignition transport also cannot differentiate connections locally and
  // remotely, so a bidirectional bridge will always subscribe.
  // \TODO(mjcarroll) Add transport APIs for correctly querying number
  // of remote subscriptions
  if (this->ign_publisher_.Valid() && this->ign_publisher_.HasConnections()) {
    return 1;
  }
  return 0;
}

bool BridgeRosToIgn::HasPublisher() const
{
  // Return Ignition publisher status
  return this->ign_publisher_.Valid();
}

void BridgeRosToIgn::StartPublisher()
{
  // Start Ignition publisher
  this->ign_publisher_ = this->factory_->create_ign_publisher(
    this->ign_node_,
    this->ign_topic_name_,
    this->publisher_queue_size_);
}

bool BridgeRosToIgn::HasSubscriber() const
{
  // Return ROS subscriber status
  return this->ros_subscriber_ != nullptr;
}

void BridgeRosToIgn::StartSubscriber()
{
  // Start ROS subscriber
  this->ros_subscriber_ = this->factory_->create_ros_subscriber(
    this->ros_node_,
    this->ros_topic_name_,
    this->subscriber_queue_size_,
    this->ign_publisher_);
}

void BridgeRosToIgn::StopSubscriber()
{
  // Stop ROS subscriber
  this->ros_subscriber_.reset();
}

}  // namespace ros_ign_bridge
