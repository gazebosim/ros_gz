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

#ifndef ROS_IGN_BRIDGE__CONVERT__STD_MSGS_HPP_
#define ROS_IGN_BRIDGE__CONVERT__STD_MSGS_HPP_

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>

// Ignition messages
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/color.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/header.pb.h>
#include <ignition/msgs/int32.pb.h>
#include <ignition/msgs/uint32.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <ros_ign_bridge/convert_decl.hpp>

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::msg::Bool & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::ColorRGBA & ros_msg,
  ignition::msgs::Color & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Color & ign_msg,
  std_msgs::msg::ColorRGBA & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Empty & ros_msg,
  ignition::msgs::Empty & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Empty & ign_msg,
  std_msgs::msg::Empty & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::UInt32 & ros_msg,
  ignition::msgs::UInt32 & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::UInt32 & ign_msg,
  std_msgs::msg::UInt32 & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::msg::Float32 & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Float64 & ros_msg,
  ignition::msgs::Double & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Double & ign_msg,
  std_msgs::msg::Float64 & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Header & ros_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::msg::Header & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::msg::Int32 & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::msg::String & ros_msg);

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT__STD_MSGS_HPP_
