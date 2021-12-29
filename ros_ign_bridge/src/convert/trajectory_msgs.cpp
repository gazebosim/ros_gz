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
#include "ros_ign_bridge/convert/trajectory_msgs.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const trajectory_msgs::msg::JointTrajectoryPoint & ros_msg,
  ignition::msgs::JointTrajectoryPoint & ign_msg)
{
  for (const auto & ros_position : ros_msg.positions) {
    ign_msg.add_positions(ros_position);
  }
  for (const auto & ros_velocity : ros_msg.velocities) {
    ign_msg.add_velocities(ros_velocity);
  }
  for (const auto & ros_acceleration : ros_msg.accelerations) {
    ign_msg.add_accelerations(ros_acceleration);
  }
  for (const auto & ros_effort : ros_msg.effort) {
    ign_msg.add_effort(ros_effort);
  }

  ignition::msgs::Duration * ign_duration = ign_msg.mutable_time_from_start();
  ign_duration->set_sec(ros_msg.time_from_start.sec);
  ign_duration->set_nsec(ros_msg.time_from_start.nanosec);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::JointTrajectoryPoint & ign_msg,
  trajectory_msgs::msg::JointTrajectoryPoint & ros_msg)
{
  for (auto i = 0; i < ign_msg.positions_size(); ++i) {
    ros_msg.positions.push_back(ign_msg.positions(i));
  }
  for (auto i = 0; i < ign_msg.velocities_size(); ++i) {
    ros_msg.velocities.push_back(ign_msg.velocities(i));
  }
  for (auto i = 0; i < ign_msg.accelerations_size(); ++i) {
    ros_msg.accelerations.push_back(ign_msg.accelerations(i));
  }
  for (auto i = 0; i < ign_msg.effort_size(); ++i) {
    ros_msg.effort.push_back(ign_msg.effort(i));
  }

  ros_msg.time_from_start = rclcpp::Duration(
    ign_msg.time_from_start().sec(),
    ign_msg.time_from_start().nsec());
}

template<>
void
convert_ros_to_ign(
  const trajectory_msgs::msg::JointTrajectory & ros_msg,
  ignition::msgs::JointTrajectory & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  for (const auto & ros_joint_name : ros_msg.joint_names) {
    ign_msg.add_joint_names(ros_joint_name);
  }

  for (const auto & ros_point : ros_msg.points) {
    ignition::msgs::JointTrajectoryPoint * ign_point = ign_msg.add_points();
    convert_ros_to_ign(ros_point, (*ign_point));
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::JointTrajectory & ign_msg,
  trajectory_msgs::msg::JointTrajectory & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  for (auto i = 0; i < ign_msg.joint_names_size(); ++i) {
    ros_msg.joint_names.push_back(ign_msg.joint_names(i));
  }

  for (auto i = 0; i < ign_msg.points_size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint ros_point;
    convert_ign_to_ros(ign_msg.points(i), ros_point);
    ros_msg.points.push_back(ros_point);
  }
}

}  // namespace ros_ign_bridge
