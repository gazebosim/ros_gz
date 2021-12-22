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
#include "trajectory_msgs.hpp"

#include "ros_ign_bridge/convert/trajectory_msgs.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__trajectory_msgs(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if ((ros_type_name == "trajectory_msgs/msg/JointTrajectory" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.JointTrajectory")
  {
    return std::make_shared<
      Factory<
        trajectory_msgs::msg::JointTrajectory,
        ignition::msgs::JointTrajectory
      >
    >("trajectory_msgs/msg/JointTrajectory", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  trajectory_msgs::msg::JointTrajectoryPoint,
  ignition::msgs::JointTrajectoryPoint
>::convert_ros_to_ign(
  const trajectory_msgs::msg::JointTrajectoryPoint & ros_msg,
  ignition::msgs::JointTrajectoryPoint & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  trajectory_msgs::msg::JointTrajectoryPoint,
  ignition::msgs::JointTrajectoryPoint
>::convert_ign_to_ros(
  const ignition::msgs::JointTrajectoryPoint & ign_msg,
  trajectory_msgs::msg::JointTrajectoryPoint & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  trajectory_msgs::msg::JointTrajectory,
  ignition::msgs::JointTrajectory
>::convert_ros_to_ign(
  const trajectory_msgs::msg::JointTrajectory & ros_msg,
  ignition::msgs::JointTrajectory & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  trajectory_msgs::msg::JointTrajectory,
  ignition::msgs::JointTrajectory
>::convert_ign_to_ros(
  const ignition::msgs::JointTrajectory & ign_msg,
  trajectory_msgs::msg::JointTrajectory & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}


}  // namespace ros_ign_bridge
