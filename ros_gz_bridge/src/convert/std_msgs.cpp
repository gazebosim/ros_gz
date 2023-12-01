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

#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/builtin_interfaces.hpp"
#include "ros_gz_bridge/convert/std_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Bool & ros_msg,
  gz::msgs::Boolean & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_msg,
  std_msgs::msg::Bool & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::ColorRGBA & ros_msg,
  gz::msgs::Color & gz_msg)
{
  gz_msg.set_r(ros_msg.r);
  gz_msg.set_g(ros_msg.g);
  gz_msg.set_b(ros_msg.b);
  gz_msg.set_a(ros_msg.a);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Color & gz_msg,
  std_msgs::msg::ColorRGBA & ros_msg)
{
  ros_msg.r = gz_msg.r();
  ros_msg.g = gz_msg.g();
  ros_msg.b = gz_msg.b();
  ros_msg.a = gz_msg.a();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Empty &,
  gz::msgs::Empty &)
{
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Empty &,
  std_msgs::msg::Empty &)
{
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::UInt32 & ros_msg,
  gz::msgs::UInt32 & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::UInt32 & gz_msg,
  std_msgs::msg::UInt32 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Float32 & ros_msg,
  gz::msgs::Float & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Float & gz_msg,
  std_msgs::msg::Float32 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Float64 & ros_msg,
  gz::msgs::Double & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Double & gz_msg,
  std_msgs::msg::Float64 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Header & ros_msg,
  gz::msgs::Header & gz_msg)
{
  convert_ros_to_gz(ros_msg.stamp, *gz_msg.mutable_stamp());
  auto newPair = gz_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros_msg.frame_id);
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Int32 & ros_msg,
  gz::msgs::Int32 & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Int32 & gz_msg,
  std_msgs::msg::Int32 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::msg::Header & ros_msg)
{
  convert_gz_to_ros(gz_msg.stamp(), ros_msg.stamp);
  for (auto i = 0; i < gz_msg.data_size(); ++i) {
    auto aPair = gz_msg.data(i);
    if (aPair.key() == "frame_id" && aPair.value_size() > 0) {
      ros_msg.frame_id = frame_id_gz_to_ros(aPair.value(0));
    }
  }
}

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::String & ros_msg,
  gz::msgs::StringMsg & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::StringMsg & gz_msg,
  std_msgs::msg::String & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

}  // namespace ros_gz_bridge
