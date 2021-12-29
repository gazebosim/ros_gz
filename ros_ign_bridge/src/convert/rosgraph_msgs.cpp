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

#include <rclcpp/time.hpp>

#include "convert/utils.hpp"
#include "ros_ign_bridge/convert/rosgraph_msgs.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::msg::Clock & ros_msg)
{
  ros_msg.clock = rclcpp::Time(ign_msg.sim().sec(), ign_msg.sim().nsec());
}

template<>
void
convert_ros_to_ign(
  const rosgraph_msgs::msg::Clock & ros_msg,
  ignition::msgs::Clock & ign_msg)
{
  ign_msg.mutable_sim()->set_sec(ros_msg.clock.sec);
  ign_msg.mutable_sim()->set_nsec(ros_msg.clock.nanosec);
}

}  // namespace ros_ign_bridge
