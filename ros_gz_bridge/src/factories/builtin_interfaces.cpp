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

#include "factories/builtin_interfaces.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/builtin_interfaces.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__builtin_interfaces(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if (
    (ros_type_name == "builtin_interfaces/msg/Time" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Time")
  {
    return std::make_shared<
      Factory<
        builtin_interfaces::msg::Time,
        ignition::msgs::Time
      >
    >("builtin_interfaces/msg/Time", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  builtin_interfaces::msg::Time,
  ignition::msgs::Time
>::convert_ros_to_ign(
  const builtin_interfaces::msg::Time & ros_msg,
  ignition::msgs::Time & ign_msg)
{
  ros_gz_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  builtin_interfaces::msg::Time,
  ignition::msgs::Time
>::convert_ign_to_ros(
  const ignition::msgs::Time & ign_msg,
  builtin_interfaces::msg::Time & ros_msg)
{
  ros_gz_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}
}  // namespace ros_gz_bridge
