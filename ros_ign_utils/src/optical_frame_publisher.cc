/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "ros_ign_utils/optical_frame_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <rclcpp/rclcpp.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

namespace ros_ign_utils
{
struct OpticalFramePublisher::Impl
{
  void CheckSubscribers();

  void ImageConnect();

  void CameraInfoConnect();

  void UpdateImageFrame(const sensor_msgs::msg::Image & msg);

  void UpdateCameraInfoFrame(const sensor_msgs::msg::CameraInfo & msg);

  void PublishTF(const std::string & frame, const std::string & child_frame);

  /// \brief Interface for creating publications/subscriptions
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics;

  /// \brief timer with callback to check for publisher subscription count
  rclcpp::TimerBase::SharedPtr timer;

  /// \brief Static transform broadcaster that broadcasts the optical frame id
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcasterStatic;

  /// \brief Publisher for the image optical frame topic
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

  /// \brief Publisher for the camera info optical frame topic
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;

  /// \brief Subscriber for the original image topic
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

  /// \brief Subscriber for the original camera_info topic
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

  /// \brief Publish camera info or just the image
  bool publish_camera_info;

  /// \brief New frame id of published messages
  std::string new_frame_id;
};

void OpticalFramePublisher::Impl::CheckSubscribers()
{
  if (image_pub->get_subscription_count() > 0u) {
    ImageConnect();
  }

  if (info_pub && info_pub->get_subscription_count() > 0u) {
    CameraInfoConnect();
  }
}

void OpticalFramePublisher::Impl::ImageConnect()
{
  // Skip connecting if publisher is already created (non-nullptr)
  if (image_pub) {
    return;
  }

  image_sub = rclcpp::create_subscription < sensor_msgs::msg::Image > (
    node_topics,
    "input/image", 10,
    std::bind(&OpticalFramePublisher::Impl::UpdateImageFrame, this, _1));
}

void OpticalFramePublisher::Impl::CameraInfoConnect()
{
  // Skip connecting if publisher is already created (non-nullptr)
  if (info_pub) {
    return;
  }

  info_sub = rclcpp::create_subscription < sensor_msgs::msg::CameraInfo > (
    node_topics,
    "input/camera_info", 10,
    std::bind(&OpticalFramePublisher::Impl::UpdateCameraInfoFrame, this, _1));
}

void OpticalFramePublisher::Impl::UpdateImageFrame(const sensor_msgs::msg::Image & msg)
{
  if (image_pub->get_subscription_count() == 0u && image_sub) {
    image_sub.reset();
    return;
  }
  if (new_frame_id.empty()) {
    new_frame_id = msg.header.frame_id + "_optical";
    PublishTF(msg.header.frame_id, new_frame_id);
  }
  auto m = msg;
  m.header.frame_id = new_frame_id;
  image_pub->publish(m);
}

void OpticalFramePublisher::Impl::UpdateCameraInfoFrame(const sensor_msgs::msg::CameraInfo & msg)
{
  if (info_pub->get_subscription_count() == 0u && info_sub) {
    info_sub.reset();
    return;
  }
  if (new_frame_id.empty()) {
    new_frame_id = msg.header.frame_id + "_optical";
    PublishTF(msg.header.frame_id, new_frame_id);
  }
  auto m = msg;
  m.header.frame_id = new_frame_id;
  info_pub->publish(m);
}

void OpticalFramePublisher::Impl::PublishTF(
  const std::string & frame,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped tfStamped;
  tfStamped.header.frame_id = frame;
  tfStamped.child_frame_id = child_frame;
  tfStamped.transform.translation.x = 0.0;
  tfStamped.transform.translation.y = 0.0;
  tfStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  // converts x forward to z forward
  q.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0);
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();
  tfBroadcasterStatic->sendTransform(tfStamped);
}


OpticalFramePublisher::OpticalFramePublisher(const rclcpp::NodeOptions & options)
: Node("optical_frame_publisher", options),
  dataPtr(std::make_unique < Impl > ())
{
  dataPtr->node_topics = this->get_node_topics_interface();

  dataPtr->tfBroadcasterStatic =
    std::make_unique < tf2_ros::StaticTransformBroadcaster > (*this);

  dataPtr->publish_camera_info = this->declare_parameter("publish_camera_info", true);

  dataPtr->image_pub = this->create_publisher < sensor_msgs::msg::Image > (
    "output/image", 10);

  if (dataPtr->publish_camera_info) {
    dataPtr->info_pub = this->create_publisher < sensor_msgs::msg::CameraInfo > (
      "output/camera_info", 10);
  }

  dataPtr->timer = this->create_wall_timer(
    100ms,
    std::bind(&OpticalFramePublisher::Impl::CheckSubscribers, dataPtr.get()));
}
}  // namespace ros_ign_utils
