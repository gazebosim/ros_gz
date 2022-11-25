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

#ifndef ROS_GZ_BRIDGE__CONVERT__GPS_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__GPS_MSGS_HPP_

// Gazebo Msgs
#include <gz/msgs/navsat.pb.h>

// ROS 2 messages
#include <gps_msgs/msg/gps_fix.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{
template<>
void
convert_ros_to_gz(
  const gps_msgs::msg::GPSFix & ros_msg,
  gz::msgs::NavSat & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  gps_msgs::msg::GPSFix & ros_msg);
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__GPS_MSGS_HPP_
