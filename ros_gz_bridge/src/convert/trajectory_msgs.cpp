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
#include "ros_gz_bridge/convert/trajectory_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const trajectory_msgs::msg::JointTrajectoryPoint & ros_msg,
  gz::msgs::JointTrajectoryPoint & gz_msg)
{
  for (const auto & ros_position : ros_msg.positions) {
    gz_msg.add_positions(ros_position);
  }
  for (const auto & ros_velocity : ros_msg.velocities) {
    gz_msg.add_velocities(ros_velocity);
  }
  for (const auto & ros_acceleration : ros_msg.accelerations) {
    gz_msg.add_accelerations(ros_acceleration);
  }
  for (const auto & ros_effort : ros_msg.effort) {
    gz_msg.add_effort(ros_effort);
  }

  gz::msgs::Duration * gz_duration = gz_msg.mutable_time_from_start();
  gz_duration->set_sec(ros_msg.time_from_start.sec);
  gz_duration->set_nsec(ros_msg.time_from_start.nanosec);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::JointTrajectoryPoint & gz_msg,
  trajectory_msgs::msg::JointTrajectoryPoint & ros_msg)
{
  for (auto i = 0; i < gz_msg.positions_size(); ++i) {
    ros_msg.positions.push_back(gz_msg.positions(i));
  }
  for (auto i = 0; i < gz_msg.velocities_size(); ++i) {
    ros_msg.velocities.push_back(gz_msg.velocities(i));
  }
  for (auto i = 0; i < gz_msg.accelerations_size(); ++i) {
    ros_msg.accelerations.push_back(gz_msg.accelerations(i));
  }
  for (auto i = 0; i < gz_msg.effort_size(); ++i) {
    ros_msg.effort.push_back(gz_msg.effort(i));
  }

  ros_msg.time_from_start = rclcpp::Duration(
    gz_msg.time_from_start().sec(),
    gz_msg.time_from_start().nsec());
}

template<>
void
convert_ros_to_gz(
  const trajectory_msgs::msg::JointTrajectory & ros_msg,
  gz::msgs::JointTrajectory & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  for (const auto & ros_joint_name : ros_msg.joint_names) {
    gz_msg.add_joint_names(ros_joint_name);
  }

  for (const auto & ros_point : ros_msg.points) {
    gz::msgs::JointTrajectoryPoint * gz_point = gz_msg.add_points();
    convert_ros_to_gz(ros_point, (*gz_point));
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::JointTrajectory & gz_msg,
  trajectory_msgs::msg::JointTrajectory & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for (auto i = 0; i < gz_msg.joint_names_size(); ++i) {
    ros_msg.joint_names.push_back(gz_msg.joint_names(i));
  }

  for (auto i = 0; i < gz_msg.points_size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint ros_point;
    convert_gz_to_ros(gz_msg.points(i), ros_point);
    ros_msg.points.push_back(ros_point);
  }
}

}  // namespace ros_gz_bridge
