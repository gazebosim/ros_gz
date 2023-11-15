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
#include "ros_gz_bridge/convert/rosgraph_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_gz_to_ros(
  const gz::msgs::Clock & gz_msg,
  rosgraph_msgs::msg::Clock & ros_msg)
{
  ros_msg.clock = rclcpp::Time(gz_msg.sim().sec(), gz_msg.sim().nsec());
}

template<>
void
convert_ros_to_gz(
  const rosgraph_msgs::msg::Clock & ros_msg,
  gz::msgs::Clock & gz_msg)
{
  gz_msg.mutable_sim()->set_sec(ros_msg.clock.sec);
  gz_msg.mutable_sim()->set_nsec(ros_msg.clock.nanosec);
}

}  // namespace ros_gz_bridge
