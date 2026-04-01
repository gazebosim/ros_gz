// Copyright 2026 Honu Robotics
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

#ifndef ROS_GZ_BRIDGE__CONVERT__MARINE_ACOUSTIC_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__MARINE_ACOUSTIC_MSGS_HPP_

// Gazebo Msgs
#include <gz/msgs/dvl_velocity_tracking.pb.h>

// ROS 2 messages
#include <marine_acoustic_msgs/msg/dvl.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{
// marine_acoustic_msgs
template<>
void
convert_ros_to_gz(
  const marine_acoustic_msgs::msg::Dvl & ros_msg,
  gz::msgs::DVLVelocityTracking & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::DVLVelocityTracking & gz_msg,
  marine_acoustic_msgs::msg::Dvl & ros_msg);
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__MARINE_ACOUSTIC_MSGS_HPP_
