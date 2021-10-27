// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <exception>
#include <ros/console.h>

#include "ros_ign_bridge/convert.hpp"
#include "convert/frame_id.hpp"

namespace ros_ign_bridge
{
template<>
void
convert_ros_to_ign(
  const geometry_msgs::Quaternion & ros_msg,
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
  geometry_msgs::Quaternion & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
  ros_msg.w = ign_msg.w();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Vector3 & ros_msg,
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
  geometry_msgs::Vector3 & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Point & ros_msg,
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
  geometry_msgs::Point & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.position, *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.orientation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.position);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.orientation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::PoseArray & ros_msg,
  ignition::msgs::Pose_V & ign_msg)
{
  ign_msg.clear_pose();
  for (auto const &t : ros_msg.poses)
  {
    auto p = ign_msg.add_pose();
    convert_ros_to_ign(t, *p);
  }

  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  geometry_msgs::PoseArray & ros_msg)
{
  ros_msg.poses.clear();
  for (auto const &p : ign_msg.pose())
  {
    geometry_msgs::Pose pose;
    convert_ign_to_ros(p, pose);
    ros_msg.poses.push_back(pose);
  }
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose, ign_msg);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.pose);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.translation , *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.rotation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.translation);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.rotation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::TransformStamped & ros_msg,
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
  geometry_msgs::TransformStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.transform);
  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_ign(
  const tf2_msgs::TFMessage & ros_msg,
  ignition::msgs::Pose_V & ign_msg)
{
  ign_msg.clear_pose();
  for (auto const &t : ros_msg.transforms)
  {
    auto p = ign_msg.add_pose();
    convert_ros_to_ign(t, *p);
  }

  if (!ros_msg.transforms.empty())
  {
    convert_ros_to_ign(ros_msg.transforms[0].header,
        (*ign_msg.mutable_header()));
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  tf2_msgs::TFMessage & ros_msg)
{
  ros_msg.transforms.clear();
  for (auto const &p : ign_msg.pose())
  {
    geometry_msgs::TransformStamped tf;
    convert_ign_to_ros(p, tf);
    ros_msg.transforms.push_back(tf);
  }
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Twist & ros_msg,
  ignition::msgs::Twist & ign_msg)
{
  convert_ros_to_ign(ros_msg.linear,  (*ign_msg.mutable_linear()));
  convert_ros_to_ign(ros_msg.angular, (*ign_msg.mutable_angular()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros_msg)
{
  convert_ign_to_ros(ign_msg.linear(), ros_msg.linear);
  convert_ign_to_ros(ign_msg.angular(), ros_msg.angular);
}
}  // namespace ros_ign_bridge

