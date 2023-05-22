// Copyright 2023 Rudis Laboratories LLC
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

#ifndef ROS_GZ_BRIDGE__CONVERT__ACTUATOR_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__ACTUATOR_MSGS_HPP_

// Gazebo Msgs
#include <ignition/msgs/actuators.pb.h>

// ROS 2 messages
#include <actuator_msgs/msg/actuators.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{
// actuator_msgs
template<>
void
convert_ros_to_gz(
  const actuator_msgs::msg::Actuators & ros_msg,
  ignition::msgs::Actuators & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Actuators & gz_msg,
  actuator_msgs::msg::Actuators & ros_msg);

}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__ACTUATOR_MSGS_HPP_
