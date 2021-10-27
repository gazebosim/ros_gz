// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef ROS_IGN_BRIDGE__FACTORIES__STD_MSGS_HPP_
#define ROS_IGN_BRIDGE__FACTORIES__STD_MSGS_HPP_

// ROS messages
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Ignition messages
#include <ignition/msgs.hh>

#include "factory.hpp"

#include <memory>
#include <string>

namespace ros_ign_bridge 
{
std::shared_ptr<FactoryInterface>
get_factory_std_msgs(
            const std::string & ros_type_name,
            const std::string & ign_type_name);

template<>
void
Factory<
  std_msgs::Bool,
  ignition::msgs::Boolean
>::convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg);

template<>
void
Factory<
  std_msgs::Bool,
  ignition::msgs::Boolean
>::convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg);

template<>
void
Factory<
  std_msgs::ColorRGBA,
  ignition::msgs::Color
>::convert_ros_to_ign(
  const std_msgs::ColorRGBA & ros_msg,
  ignition::msgs::Color & ign_msg);

template<>
void
Factory<
  std_msgs::ColorRGBA,
  ignition::msgs::Color
>::convert_ign_to_ros(
  const ignition::msgs::Color & ign_msg,
  std_msgs::ColorRGBA & ros_msg);

template<>
void
Factory<
  std_msgs::Empty,
  ignition::msgs::Empty
>::convert_ros_to_ign(
  const std_msgs::Empty & ros_msg,
  ignition::msgs::Empty & ign_msg);

template<>
void
Factory<
  std_msgs::Empty,
  ignition::msgs::Empty
>::convert_ign_to_ros(
  const ignition::msgs::Empty & ign_msg,
  std_msgs::Empty & ros_msg);

template<>
void
Factory<
  std_msgs::Int32,
  ignition::msgs::Int32
>::convert_ros_to_ign(
  const std_msgs::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg);

template<>
void
Factory<
  std_msgs::Int32,
  ignition::msgs::Int32
>::convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::Int32 & ros_msg);

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg);

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg);

template<>
void
Factory<
  std_msgs::Float64,
  ignition::msgs::Double
>::convert_ros_to_ign(
  const std_msgs::Float64 & ros_msg,
  ignition::msgs::Double & ign_msg);

template<>
void
Factory<
  std_msgs::Float64,
  ignition::msgs::Double
>::convert_ign_to_ros(
  const ignition::msgs::Double & ign_msg,
  std_msgs::Float64 & ros_msg);

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg);

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg);
}  // namespace ros_ign_bridge
#endif  // ROS_IGN_BRIDGE__FACTORIES__STD_MSGS_HPP_
