// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef ROS_GZ_SIM__SIMULATION_INTERFACES_UTILS_HPP_
#define ROS_GZ_SIM__SIMULATION_INTERFACES_UTILS_HPP_

#include <gz/math/Pose3.hh>
#include <simulation_interfaces/msg/entity_state.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
void ConvertPose(const gz::math::Pose3d & gz_pose, geometry_msgs::msg::Pose & ros_pose);
geometry_msgs::msg::Pose ConvertPose(const gz::math::Pose3d & gz_pose);

void ConvertPose(const geometry_msgs::msg::Pose & ros_pose, gz::math::Pose3d & gz_pose);
gz::math::Pose3d ConvertPose(const geometry_msgs::msg::Pose & ros_pose);

void ConvertVector3(const gz::math::Vector3d & gz_v, geometry_msgs::msg::Vector3 & ros_v);
geometry_msgs::msg::Vector3  ConvertVector3(const gz::math::Vector3d& gz_v);

void ConvertVector3(const geometry_msgs::msg::Vector3 & ros_v, gz::math::Vector3d & gz_v);
gz::math::Vector3d ConvertVector3(const geometry_msgs::msg::Vector3 & ros_v);

}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif
