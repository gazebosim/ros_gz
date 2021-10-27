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

#ifndef ROS_IGN_BRIDGE__FACTORIES__NAV_MSGS_HPP_
#define ROS_IGN_BRIDGE__FACTORIES__NAV_MSGS_HPP_

// ROS messages
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

// Ignition messages
#include <ignition/msgs.hh>

#include "factory.hpp"

#include <memory>
#include <string>

namespace ros_ign_bridge 
{
std::shared_ptr<FactoryInterface>
get_factory_nav_msgs(
            const std::string & ros_type_name,
            const std::string & ign_type_name);
template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  ignition::msgs::OccupancyGrid
>::convert_ros_to_ign(
  const nav_msgs::OccupancyGrid & ros_msg,
  ignition::msgs::OccupancyGrid & ign_msg);

template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  ignition::msgs::OccupancyGrid
>::convert_ign_to_ros(
  const ignition::msgs::OccupancyGrid & ign_msg,
  nav_msgs::OccupancyGrid & ros_msg);

template<>
void
Factory<
  nav_msgs::Odometry,
  ignition::msgs::Odometry
>::convert_ros_to_ign(
  const nav_msgs::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg);

template<>
void
Factory<
  nav_msgs::Odometry,
  ignition::msgs::Odometry
>::convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros_msg);
}  // namespace ros_ign_bridge
#endif  // ROS_IGN_BRIDGE__FACTORIES__NAV_MSGS_HPP_
