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

#include <memory>
#include <string>

#include "bridge.hpp"
#include "get_factory.hpp"

namespace ros_ign_bridge
{
Bridge::Bridge(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  size_t publisher_queue_size,
  size_t subscriber_queue_size,
  bool is_lazy)
: ros_node_(ros_node),
  ign_node_(ign_node),
  ros_type_name_(ros_type_name),
  ros_topic_name_(ros_topic_name),
  ign_type_name_(ign_type_name),
  ign_topic_name_(ign_topic_name),
  subscriber_queue_size_(subscriber_queue_size),
  publisher_queue_size_(publisher_queue_size),
  is_lazy_(is_lazy),
  factory_(get_factory(ros_type_name_, ign_type_name_))
{
}

Bridge::~Bridge() = default;

bool Bridge::IsLazy() const
{
  return is_lazy_;
}

void Bridge::Start()
{
  if (!this->HasPublisher()) {
    this->StartPublisher();
  }

  if (!this->IsLazy() && !this->HasSubscriber()) {
    this->StartSubscriber();
  }
}

void Bridge::Spin()
{
  if (!this->IsLazy()) {
    return;
  }

  if (this->HasSubscriber() && this->NumSubscriptions() == 0) {
    RCLCPP_DEBUG(
      this->ros_node_->get_logger(),
      "Bridge [%s] - No subscriptions found, stopping bridge",
      ros_topic_name_.c_str());
    this->StopSubscriber();
  } else if (!this->HasSubscriber() && this->NumSubscriptions() > 0) {
    RCLCPP_DEBUG(
      this->ros_node_->get_logger(),
      "Bridge [%s] - Subscriptions found, starting bridge",
      ros_topic_name_.c_str());
    this->StartSubscriber();
  }
}
}  // namespace ros_ign_bridge
