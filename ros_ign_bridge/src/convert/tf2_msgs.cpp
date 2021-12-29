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
#include "ros_ign_bridge/convert/tf2_msgs.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const tf2_msgs::msg::TFMessage & ros_msg,
  ignition::msgs::Pose_V & ign_msg)
{
  ign_msg.clear_pose();
  for (auto const & t : ros_msg.transforms) {
    auto p = ign_msg.add_pose();
    convert_ros_to_ign(t, *p);
  }

  if (!ros_msg.transforms.empty()) {
    convert_ros_to_ign(
      ros_msg.transforms[0].header,
      (*ign_msg.mutable_header()));
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  tf2_msgs::msg::TFMessage & ros_msg)
{
  ros_msg.transforms.clear();
  for (auto const & p : ign_msg.pose()) {
    geometry_msgs::msg::TransformStamped tf;
    convert_ign_to_ros(p, tf);
    ros_msg.transforms.push_back(tf);
  }
}

}  // namespace ros_ign_bridge
