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

#include "factories/std_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_ign_bridge/convert/std_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__std_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  // mapping from string to specialized template
  if ((ros_type_name == "std_msgs/msg/Bool" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Boolean" || gz_type_name == "ignition.msgs.Boolean"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Bool,
        ignition::msgs::Boolean
      >
    >("std_msgs/msg/Bool", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/ColorRGBA" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Color" || gz_type_name == "ignition.msgs.Color"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::ColorRGBA,
        ignition::msgs::Color
      >
    >("std_msgs/msg/ColorRGBA", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/Empty" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Empty" || gz_type_name == "ignition.msgs.Empty"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Empty,
        ignition::msgs::Empty
      >
    >("std_msgs/msg/Empty", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/Float32" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Float" || gz_type_name == "ignition.msgs.Float"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Float32,
        ignition::msgs::Float
      >
    >("std_msgs/msg/Float32", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/Float64" || ros_type_name == "")
      && (gz_type_name == "gz.msgs.Double" || gz_type_name == "ignition.msgs.Double"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Float64,
        ignition::msgs::Double
      >
    >("std_msgs/msg/Float64", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/Header" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Header" || gz_type_name == "ignition.msgs.Header"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Header,
        ignition::msgs::Header
      >
    >("std_msgs/msg/Header", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/Int32" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Int32" || gz_type_name == "ignition.msgs.Int32"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Int32,
        ignition::msgs::Int32
      >
    >("std_msgs/msg/Int32", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/UInt32" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.UInt32" || gz_type_name == "ignition.msgs.UInt32"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::UInt32,
        ignition::msgs::UInt32
      >
    >("std_msgs/msg/Int32", gz_type_name);
  }
  if ((ros_type_name == "std_msgs/msg/String" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.StringMsg" || gz_type_name == "ignition.msgs.StringMsg"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::String,
        ignition::msgs::StringMsg
      >
    >("std_msgs/msg/String", gz_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  std_msgs::msg::Bool,
  ignition::msgs::Boolean
>::convert_ros_to_gz(
  const std_msgs::msg::Bool & ros_msg,
  ignition::msgs::Boolean & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Bool,
  ignition::msgs::Boolean
>::convert_gz_to_ros(
  const ignition::msgs::Boolean & gz_msg,
  std_msgs::msg::Bool & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::ColorRGBA,
  ignition::msgs::Color
>::convert_ros_to_gz(
  const std_msgs::msg::ColorRGBA & ros_msg,
  ignition::msgs::Color & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::ColorRGBA,
  ignition::msgs::Color
>::convert_gz_to_ros(
  const ignition::msgs::Color & gz_msg,
  std_msgs::msg::ColorRGBA & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::Empty,
  ignition::msgs::Empty
>::convert_ros_to_gz(
  const std_msgs::msg::Empty & ros_msg,
  ignition::msgs::Empty & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Empty,
  ignition::msgs::Empty
>::convert_gz_to_ros(
  const ignition::msgs::Empty & gz_msg,
  std_msgs::msg::Empty & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::Float32,
  ignition::msgs::Float
>::convert_ros_to_gz(
  const std_msgs::msg::Float32 & ros_msg,
  ignition::msgs::Float & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Float32,
  ignition::msgs::Float
>::convert_gz_to_ros(
  const ignition::msgs::Float & gz_msg,
  std_msgs::msg::Float32 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::Float64,
  ignition::msgs::Double
>::convert_ros_to_gz(
  const std_msgs::msg::Float64 & ros_msg,
  ignition::msgs::Double & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Float64,
  ignition::msgs::Double
>::convert_gz_to_ros(
  const ignition::msgs::Double & gz_msg,
  std_msgs::msg::Float64 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::Int32,
  ignition::msgs::Int32
>::convert_ros_to_gz(
  const std_msgs::msg::Int32 & ros_msg,
  ignition::msgs::Int32 & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Int32,
  ignition::msgs::Int32
>::convert_gz_to_ros(
  const ignition::msgs::Int32 & gz_msg,
  std_msgs::msg::Int32 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::UInt32,
  ignition::msgs::UInt32
>::convert_ros_to_gz(
  const std_msgs::msg::UInt32 & ros_msg,
  ignition::msgs::UInt32 & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::UInt32,
  ignition::msgs::UInt32
>::convert_gz_to_ros(
  const ignition::msgs::UInt32 & gz_msg,
  std_msgs::msg::UInt32 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::Header,
  ignition::msgs::Header
>::convert_ros_to_gz(
  const std_msgs::msg::Header & ros_msg,
  ignition::msgs::Header & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Header,
  ignition::msgs::Header
>::convert_gz_to_ros(
  const ignition::msgs::Header & gz_msg,
  std_msgs::msg::Header & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ros_to_gz(
  const std_msgs::msg::String & ros_msg,
  ignition::msgs::StringMsg & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_gz_to_ros(
  const ignition::msgs::StringMsg & gz_msg,
  std_msgs::msg::String & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
