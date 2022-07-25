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

#ifndef ROS_IGN_BRIDGE__CONVERT__NAV_MSGS_HPP_
#define ROS_IGN_BRIDGE__CONVERT__NAV_MSGS_HPP_

// Gazebo Msgs
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/odometry_with_covariance.pb.h>

// ROS 2 messages
#include <nav_msgs/msg/odometry.hpp>

#include <ros_ign_bridge/convert_decl.hpp>

namespace ros_ign_bridge
{
// nav_msgs
template<>
void
convert_ros_to_ign(
  const nav_msgs::msg::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::msg::Odometry & ros_msg);

template<>
void
convert_ros_to_ign(
  const nav_msgs::msg::Odometry & ros_msg,
  ignition::msgs::OdometryWithCovariance & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::OdometryWithCovariance & ign_msg,
  nav_msgs::msg::Odometry & ros_msg);

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT__NAV_MSGS_HPP_
