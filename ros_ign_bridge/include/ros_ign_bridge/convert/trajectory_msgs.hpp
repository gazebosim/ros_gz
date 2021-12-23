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

#ifndef ROS_IGN_BRIDGE__CONVERT__TRAJECTORY_MSGS_HPP_
#define ROS_IGN_BRIDGE__CONVERT__TRAJECTORY_MSGS_HPP_

#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Ignition messages
#include <ignition/msgs/joint_trajectory.pb.h>

#include <ros_ign_bridge/convert_decl.hpp>

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const trajectory_msgs::msg::JointTrajectoryPoint & ros_msg,
  ignition::msgs::JointTrajectoryPoint & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::JointTrajectoryPoint & ign_msg,
  trajectory_msgs::msg::JointTrajectoryPoint & ros_msg);

template<>
void
convert_ros_to_ign(
  const trajectory_msgs::msg::JointTrajectory & ros_msg,
  ignition::msgs::JointTrajectory & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::JointTrajectory & ign_msg,
  trajectory_msgs::msg::JointTrajectory & ros_msg);


}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT__TRAJECTORY_MSGS_HPP_
