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

#include "factories/tf2_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_ign_bridge/convert/tf2_msgs.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__tf2_msgs(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if (
    (ros_type_name == "tf2_msgs/msg/TFMessage" || ros_type_name == "") &&
    ign_type_name == "ignition.msgs.Pose_V")
  {
    return std::make_shared<
      Factory<
        tf2_msgs::msg::TFMessage,
        ignition::msgs::Pose_V
      >
    >("tf2_msgs/msg/TFMessage", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  tf2_msgs::msg::TFMessage,
  ignition::msgs::Pose_V
>::convert_ros_to_ign(
  const tf2_msgs::msg::TFMessage & ros_msg,
  ignition::msgs::Pose_V & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  tf2_msgs::msg::TFMessage,
  ignition::msgs::Pose_V
>::convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  tf2_msgs::msg::TFMessage & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
