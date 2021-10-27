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
// geometry_msgs

#include "ros_ign_bridge/convert.hpp"
#include "factories/std_msgs.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_std_msgs(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if (
    (ros_type_name == "std_msgs/Bool" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Boolean")
  {
    return std::make_shared<
      Factory<
        std_msgs::Bool,
        ignition::msgs::Boolean
      >
    >("std_msgs/Bool", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/ColorRGBA" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Color")
  {
    return std::make_shared<
      Factory<
        std_msgs::ColorRGBA,
        ignition::msgs::Color
      >
    >("std_msgs/ColorRGBA", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Empty" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Empty")
  {
    return std::make_shared<
      Factory<
        std_msgs::Empty,
        ignition::msgs::Empty
      >
    >("std_msgs/Empty", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Int32" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Int32")
  {
    return std::make_shared<
      Factory<
        std_msgs::Int32,
        ignition::msgs::Int32
      >
    >("std_msgs/Int32", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Float32" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Float")
  {
    return std::make_shared<
      Factory<
        std_msgs::Float32,
        ignition::msgs::Float
      >
    >("std_msgs/Float32", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Float64" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Double")
  {
    return std::make_shared<
      Factory<
        std_msgs::Float64,
        ignition::msgs::Double
      >
    >("std_msgs/Float64", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Header" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Header")
  {
    return std::make_shared<
      Factory<
        std_msgs::Header,
        ignition::msgs::Header
      >
    >("std_msgs/Header", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/String" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.StringMsg")
  {
    return std::make_shared<
      Factory<
        std_msgs::String,
        ignition::msgs::StringMsg
      >
    >("std_msgs/String", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  std_msgs::Bool,
  ignition::msgs::Boolean
>::convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Bool,
  ignition::msgs::Boolean
>::convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::ColorRGBA,
  ignition::msgs::Color
>::convert_ros_to_ign(
  const std_msgs::ColorRGBA & ros_msg,
  ignition::msgs::Color & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::ColorRGBA,
  ignition::msgs::Color
>::convert_ign_to_ros(
  const ignition::msgs::Color & ign_msg,
  std_msgs::ColorRGBA & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Empty,
  ignition::msgs::Empty
>::convert_ros_to_ign(
  const std_msgs::Empty & ros_msg,
  ignition::msgs::Empty & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Empty,
  ignition::msgs::Empty
>::convert_ign_to_ros(
  const ignition::msgs::Empty & ign_msg,
  std_msgs::Empty & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Int32,
  ignition::msgs::Int32
>::convert_ros_to_ign(
  const std_msgs::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Int32,
  ignition::msgs::Int32
>::convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::Int32 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Float64,
  ignition::msgs::Double
>::convert_ros_to_ign(
  const std_msgs::Float64 & ros_msg,
  ignition::msgs::Double & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Float64,
  ignition::msgs::Double
>::convert_ign_to_ros(
  const ignition::msgs::Double & ign_msg,
  std_msgs::Float64 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  ignition::msgs::Header & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
