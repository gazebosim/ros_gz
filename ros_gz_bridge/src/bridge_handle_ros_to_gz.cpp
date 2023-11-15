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

#include "bridge_handle_ros_to_gz.hpp"

#include <gz/transport/TopicUtils.hh>

namespace ros_gz_bridge
{

BridgeHandleRosToGz::~BridgeHandleRosToGz() = default;

size_t BridgeHandleRosToGz::NumSubscriptions() const
{
  // Return number of gazebo subscriptions
  // Gazebo publishers can only detect presence of connections, and not
  // get the actual count.
  // Gazebo transport also cannot differentiate connections locally and
  // remotely, so a bidirectional bridge will always subscribe.
  // \TODO(mjcarroll) Add transport APIs for correctly querying number
  // of remote subscriptions
  if (this->gz_publisher_.Valid() && this->gz_publisher_.HasConnections()) {
    return 1;
  }
  return 0;
}

bool BridgeHandleRosToGz::HasPublisher() const
{
  // Return Gazebo publisher status
  return this->gz_publisher_.Valid();
}

void BridgeHandleRosToGz::StartPublisher()
{
  // Start Gazebo publisher
  this->gz_publisher_ = this->factory_->create_gz_publisher(
    this->gz_node_,
    this->config_.gz_topic_name,
    this->config_.publisher_queue_size);
}

bool BridgeHandleRosToGz::HasSubscriber() const
{
  // Return ROS subscriber status
  return this->ros_subscriber_ != nullptr;
}

void BridgeHandleRosToGz::StartSubscriber()
{
  // Start ROS subscriber
  this->ros_subscriber_ = this->factory_->create_ros_subscriber(
    this->ros_node_,
    this->config_.ros_topic_name,
    this->config_.subscriber_queue_size,
    this->gz_publisher_);
}

void BridgeHandleRosToGz::StopSubscriber()
{
  // Stop ROS subscriber
  this->ros_subscriber_.reset();
}

}  // namespace ros_gz_bridge
