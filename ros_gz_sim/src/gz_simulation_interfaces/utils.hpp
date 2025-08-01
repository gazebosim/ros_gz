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

#include "gazebo_proxy.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
bool ConvertState(
  const GazeboProxy::State & gz_proxy, simulation_interfaces::msg::EntityState & state);
void ConvertPose(const gz::math::Pose3d & gz_pose, geometry_msgs::msg::Pose & ros_pose);
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif
