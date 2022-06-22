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

#include "factories/rosgraph_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/rosgraph_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__rosgraph_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "rosgraph_msgs/msg/Clock" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.Clock")
  {
    return std::make_shared<
      Factory<
        rosgraph_msgs::msg::Clock,
        ignition::msgs::Clock
      >
    >("rosgraph_msgs/msg/Clock", gz_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  rosgraph_msgs::msg::Clock,
  ignition::msgs::Clock
>::convert_ros_to_ign(
  const rosgraph_msgs::msg::Clock & ros_msg,
  ignition::msgs::Clock & gz_msg)
{
  ros_gz_bridge::convert_ros_to_ign(ros_msg, gz_msg);
}

template<>
void
Factory<
  rosgraph_msgs::msg::Clock,
  ignition::msgs::Clock
>::convert_gz_to_ros(
  const ignition::msgs::Clock & gz_msg,
  rosgraph_msgs::msg::Clock & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
