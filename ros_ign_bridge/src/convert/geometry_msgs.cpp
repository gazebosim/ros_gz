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

#include "convert/utils.hpp"
#include "ros_ign_bridge/convert/geometry_msgs.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Quaternion & ros_msg,
  ignition::msgs::Quaternion & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
  ign_msg.set_w(ros_msg.w);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::msg::Quaternion & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
  ros_msg.w = ign_msg.w();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Vector3 & ros_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::msg::Vector3 & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Point & ros_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::msg::Point & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.position, *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.orientation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::Pose & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.position);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.orientation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose, ign_msg);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::PoseStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.pose);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.translation, *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.rotation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::Transform & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.translation);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.rotation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::TransformStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.transform, ign_msg);

  auto newPair = ign_msg.mutable_header()->add_data();
  newPair->set_key("child_frame_id");
  newPair->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::TransformStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.transform);
  for (auto i = 0; i < ign_msg.header().data_size(); ++i) {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0) {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Twist & ros_msg,
  ignition::msgs::Twist & ign_msg)
{
  convert_ros_to_ign(ros_msg.linear, (*ign_msg.mutable_linear()));
  convert_ros_to_ign(ros_msg.angular, (*ign_msg.mutable_angular()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::msg::Twist & ros_msg)
{
  convert_ign_to_ros(ign_msg.linear(), ros_msg.linear);
  convert_ign_to_ros(ign_msg.angular(), ros_msg.angular);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::msg::Wrench & ros_msg,
  ignition::msgs::Wrench & ign_msg)
{
  convert_ros_to_ign(ros_msg.force, (*ign_msg.mutable_force()));
  convert_ros_to_ign(ros_msg.torque, (*ign_msg.mutable_torque()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Wrench & ign_msg,
  geometry_msgs::msg::Wrench & ros_msg)
{
  convert_ign_to_ros(ign_msg.force(), ros_msg.force);
  convert_ign_to_ros(ign_msg.torque(), ros_msg.torque);
}

}  // namespace ros_ign_bridge
