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

#include <rclcpp/time.hpp>

#include "convert/utils.hpp"
#include "ros_ign_bridge/convert/std_msgs.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::msg::Bool & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::ColorRGBA & ros_msg,
  ignition::msgs::Color & ign_msg)
{
  ign_msg.set_r(ros_msg.r);
  ign_msg.set_g(ros_msg.g);
  ign_msg.set_b(ros_msg.b);
  ign_msg.set_a(ros_msg.a);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Color & ign_msg,
  std_msgs::msg::ColorRGBA & ros_msg)
{
  ros_msg.r = ign_msg.r();
  ros_msg.g = ign_msg.g();
  ros_msg.b = ign_msg.b();
  ros_msg.a = ign_msg.a();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Empty &,
  ignition::msgs::Empty &)
{
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Empty &,
  std_msgs::msg::Empty &)
{
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::UInt32 & ros_msg,
  ignition::msgs::UInt32 & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::UInt32 & ign_msg,
  std_msgs::msg::UInt32 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::msg::Float32 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Float64 & ros_msg,
  ignition::msgs::Double & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Double & ign_msg,
  std_msgs::msg::Float64 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Header & ros_msg,
  ignition::msgs::Header & ign_msg)
{
  ign_msg.mutable_stamp()->set_sec(ros_msg.stamp.sec);
  ign_msg.mutable_stamp()->set_nsec(ros_msg.stamp.nanosec);
  auto newPair = ign_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros_msg.frame_id);
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::msg::Int32 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::msg::Header & ros_msg)
{
  ros_msg.stamp = rclcpp::Time(ign_msg.stamp().sec(), ign_msg.stamp().nsec());
  for (auto i = 0; i < ign_msg.data_size(); ++i) {
    auto aPair = ign_msg.data(i);
    if (aPair.key() == "frame_id" && aPair.value_size() > 0) {
      ros_msg.frame_id = frame_id_ign_to_ros(aPair.value(0));
    }
  }
}

template<>
void
convert_ros_to_ign(
  const std_msgs::msg::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::msg::String & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

}  // namespace ros_ign_bridge
