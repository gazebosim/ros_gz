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

#include "ros_ign_bridge/convert/builtin_interfaces.hpp"

namespace ros_ign_bridge
{
template<>
void
convert_ros_to_ign(
  const builtin_interfaces::msg::Time & ros_msg,
  ignition::msgs::Time & ign_msg)
{
  ign_msg.set_sec(ros_msg.sec);
  ign_msg.set_nsec(ros_msg.nanosec);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Time & ign_msg,
  builtin_interfaces::msg::Time & ros_msg)
{
  ros_msg.sec = ign_msg.sec();
  ros_msg.nanosec = ign_msg.nsec();
}
}  // namespace ros_ign_bridge
