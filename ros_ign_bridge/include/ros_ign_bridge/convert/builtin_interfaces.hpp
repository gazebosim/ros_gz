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

#ifndef ROS_IGN_BRIDGE__CONVERT__BUILTIN_INTERFACES_HPP_
#define ROS_IGN_BRIDGE__CONVERT__BUILTIN_INTERFACES_HPP_

#include <ignition/msgs/time.pb.h>

#include <builtin_interfaces/msg/time.hpp>

#include "ros_ign_bridge/convert_decl.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const builtin_interfaces::msg::Time & ros_msg,
  ignition::msgs::Time & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Time & ign_msg,
  builtin_interfaces::msg::Time & ros_msg);

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT__BUILTIN_INTERFACES_HPP_
