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

#include "ros1_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros1_ign_bridge
{

template<>
void
convert_1_to_ign(
  const std_msgs::String & ros1_type,
  ignition::msg::StringMsg & ign_msg)
{
  ign_msg.set_data(ros1_type.data);
}

template<>
void
convert_ign_to_1(
  const ignition::msg::StringMsg & ign_msg,
  std_msgs::String & ros1_type)
{
  ros1_type.data = ign_msg.data();
}

}  // namespace ros1_ign_bridge
