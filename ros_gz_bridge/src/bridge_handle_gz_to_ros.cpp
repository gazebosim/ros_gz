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

#include "bridge_handle_gz_to_ros.hpp"

#include <tf2/LinearMath/Quaternion.h>

namespace ros_gz_bridge
{

BridgeHandleGzToRos::BridgeHandleGzToRos(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<gz::transport::Node> gz_node,
  const BridgeConfig & config)
  : BridgeHandle(ros_node, gz_node, config)
{
  ros_node_->declare_parameter<bool>("override_timestamps_with_wall_time", false);
  ros_node_->declare_parameter<bool>("publish_optical_frame", false);
  ros_node_->declare_parameter<std::string>("override_frame_id_string", "");
  // The array of doubles are [x, y, z, roll, pitch, yaw]
  ros_node_->declare_parameter<std::vector<double>>("override_frame_transform",
      std::vector<double>{});

  ros_node_->get_parameter("override_timestamps_with_wall_time",
    gz_to_ros_parameters_.override_timestamps_with_wall_time);

  // Publish_optical_optical_frame is a convenient ROS parameter that will
  // populates the override_frame_transform and override_frame_id_string
  // params with default values for converting x-forward to z-forward optical
  // frame. Note that they can still be overridden by the user if they decide
  // to set these params individually.
  bool publish_optical_frame = false;
  ros_node_->get_parameter("publish_optical_frame",
      publish_optical_frame);
  publish_optical_frame |= config.publish_optical_frame;
  std::vector<double> frame_tf;
  if (publish_optical_frame) {
    frame_tf = {0, 0, 0, -M_PI / 2.0, 0, -M_PI / 2.0};
    gz_to_ros_parameters_.override_frame_id_suffix_string = "optical";
  }
  ros_node_->get_parameter("override_frame_transform", frame_tf);
  if (!frame_tf.empty()) {
    geometry_msgs::msg::Transform transform;
    transform.translation.x = 0.0;
    transform.translation.y = 0.0;
    transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0);
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    gz_to_ros_parameters_.override_frame_transform = transform;
  }
  ros_node_->get_parameter("override_frame_id_string",
      gz_to_ros_parameters_.override_frame_id_string);

  if (gz_to_ros_parameters_.override_frame_transform &&
      (gz_to_ros_parameters_.override_frame_id_string.empty() ||
      (gz_to_ros_parameters_.override_frame_id_suffix_string.empty()) {
    std::cerr << "The 'override_frame_id_string' parameter cannot be empty "
              << "when 'override_frame_transform' is set. Disabling "
              << "'override_frame_transform'." << std::endl;
    gz_to_ros_parameters_.override_frame_transform = false;
  }
}

BridgeHandleGzToRos::~BridgeHandleGzToRos() = default;

size_t BridgeHandleGzToRos::NumSubscriptions() const
{
  // Return number of ROS subscriptions
  size_t valid_subscriptions = 0;

  if (this->ros_publisher_ != nullptr) {
    // Use info_by_topic rather than get_subscription_count
    // to filter out potential bidirectional bridge
    auto topic_info = this->ros_node_->get_subscriptions_info_by_topic(
      this->config_.ros_topic_name);

    for (auto & topic : topic_info) {
      if (topic.node_name() == this->ros_node_->get_name()) {
        continue;
      }
      valid_subscriptions++;
    }
  }

  return valid_subscriptions;
}

bool BridgeHandleGzToRos::HasPublisher() const
{
  return this->ros_publisher_ != nullptr;
}

void BridgeHandleGzToRos::StartPublisher()
{
  // Start ROS publisher
  this->ros_publisher_ = this->factory_->create_ros_publisher(
    this->ros_node_,
    this->config_.ros_topic_name,
    this->config_.publisher_queue_size);
}

bool BridgeHandleGzToRos::HasSubscriber() const
{
  // Return Gazebo subscriber status
  return this->gz_subscriber_ != nullptr;
}

void BridgeHandleGzToRos::StartSubscriber()
{
  // Start Gazebo subscriber
  this->factory_->create_gz_subscriber(
    this->gz_node_,
    this->config_.gz_topic_name,
    this->config_.subscriber_queue_size,
    this->ros_publisher_,
    this->ros_node_,
    gz_to_ros_parameters_);

  this->gz_subscriber_ = this->gz_node_;
}

void BridgeHandleGzToRos::StopSubscriber()
{
  // Stop Gazebo subscriber
  if (!this->gz_subscriber_) {
    return;
  }

  this->gz_subscriber_->Unsubscribe(this->config_.gz_topic_name);
  this->gz_subscriber_.reset();
}

}  // namespace ros_gz_bridge
