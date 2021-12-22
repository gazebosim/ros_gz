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

#include "factory.hpp"
#include "nav_msgs.hpp"

#include "ros_ign_bridge/convert/nav_msgs.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__nav_msgs(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if ((ros_type_name == "nav_msgs/msg/Odometry" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Odometry")
  {
    return std::make_shared<
      Factory<
        nav_msgs::msg::Odometry,
        ignition::msgs::Odometry
      >
    >("nav_msgs/msg/Odometry", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  nav_msgs::msg::Odometry,
  ignition::msgs::Odometry
>::convert_ros_to_ign(
  const nav_msgs::msg::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  nav_msgs::msg::Odometry,
  ignition::msgs::Odometry
>::convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::msg::Odometry & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
