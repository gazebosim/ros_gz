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
#include "ros_ign_bridge/convert/nav_msgs.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const nav_msgs::msg::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose.pose, (*ign_msg.mutable_pose()));
  convert_ros_to_ign(ros_msg.twist.twist, (*ign_msg.mutable_twist()));

  auto childFrame = ign_msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::msg::Odometry & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg.pose(), ros_msg.pose.pose);
  convert_ign_to_ros(ign_msg.twist(), ros_msg.twist.twist);

  for (auto i = 0; i < ign_msg.header().data_size(); ++i) {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0) {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}

}  // namespace ros_ign_bridge
