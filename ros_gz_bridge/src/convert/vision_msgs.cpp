// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <limits>

#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/vision_msgs.hpp"

namespace ros_gz_bridge
{
template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection2D & ros_msg,
  gz::msgs::AnnotatedAxisAligned2DBox & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz::msgs::AxisAligned2DBox * box = new gz::msgs::AxisAligned2DBox();
  gz::msgs::Vector2d * min_corner = new gz::msgs::Vector2d();
  gz::msgs::Vector2d * max_corner = new gz::msgs::Vector2d();

  if (ros_msg.results.size() != 0) {
    auto id = ros_msg.results.at(0).hypothesis.class_id;
    gz_msg.set_label(std::stoi(id));
  }

  min_corner->set_x(ros_msg.bbox.center.position.x - ros_msg.bbox.size_x / 2);
  min_corner->set_y(ros_msg.bbox.center.position.y - ros_msg.bbox.size_y / 2);
  max_corner->set_x(ros_msg.bbox.center.position.x + ros_msg.bbox.size_x / 2);
  max_corner->set_y(ros_msg.bbox.center.position.y + ros_msg.bbox.size_y / 2);
  box->set_allocated_min_corner(min_corner);
  box->set_allocated_max_corner(max_corner);
  gz_msg.set_allocated_box(box);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::AnnotatedAxisAligned2DBox & gz_msg,
  vision_msgs::msg::Detection2D & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.results.resize(1);
  ros_msg.results.at(0).hypothesis.class_id = std::to_string(gz_msg.label());
  ros_msg.results.at(0).hypothesis.score = 1.0;

  ros_msg.bbox.center.position.x = (
    gz_msg.box().min_corner().x() + gz_msg.box().max_corner().x()
    ) / 2;
  ros_msg.bbox.center.position.y = (
    gz_msg.box().min_corner().y() + gz_msg.box().max_corner().y()
    ) / 2;
  ros_msg.bbox.size_x = gz_msg.box().max_corner().x() - gz_msg.box().min_corner().x();
  ros_msg.bbox.size_y = gz_msg.box().max_corner().y() - gz_msg.box().min_corner().y();
}

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection2DArray & ros_msg,
  gz::msgs::AnnotatedAxisAligned2DBox_V & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  for (const auto & ros_box : ros_msg.detections) {
    gz::msgs::AnnotatedAxisAligned2DBox * gz_box = gz_msg.add_annotated_box();
    convert_ros_to_gz(ros_box, *gz_box);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::AnnotatedAxisAligned2DBox_V & gz_msg,
  vision_msgs::msg::Detection2DArray & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  for (const auto & gz_box : gz_msg.annotated_box()) {
    vision_msgs::msg::Detection2D ros_box;
    convert_gz_to_ros(gz_box, ros_box);
    ros_msg.detections.push_back(ros_box);
  }
}

}  // namespace ros_gz_bridge
