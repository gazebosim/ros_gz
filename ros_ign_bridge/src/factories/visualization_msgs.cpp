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
#include "factories/visualization_msgs.hpp"

namespace ros_ign_bridge
{
std::shared_ptr<FactoryInterface>
get_factory_visualization_msgs(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if (
    (ros_type_name == "visualization_msgs/Marker" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Marker")
  {
    return std::make_shared<
      Factory<
        visualization_msgs::Marker,
        ignition::msgs::Marker
      >
    >("visualization_msgs/Marker", ign_type_name);
  }
  if (
    (ros_type_name == "visualization_msgs/MarkerArray" || ros_type_name == "") &&
     ign_type_name == "ignition.msgs.Marker_V")
  {
    return std::make_shared<
      Factory<
        visualization_msgs::MarkerArray,
        ignition::msgs::Marker_V
      >
    >("visualization_msgs/MarkerArray", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  visualization_msgs::Marker,
  ignition::msgs::Marker
>::convert_ros_to_ign(
    const visualization_msgs::Marker & ros_msg,
    ignition::msgs::Marker & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  visualization_msgs::Marker,
  ignition::msgs::Marker
>::convert_ign_to_ros(
    const ignition::msgs::Marker & ign_msg,
    visualization_msgs::Marker & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  ignition::msgs::Marker_V
>::convert_ros_to_ign(
    const visualization_msgs::MarkerArray & ros_msg,
    ignition::msgs::Marker_V & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  ignition::msgs::Marker_V
>::convert_ign_to_ros(
    const ignition::msgs::Marker_V & ign_msg,
    visualization_msgs::MarkerArray & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
