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

#include "utils.hpp"

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
bool ConvertState(
  const GazeboProxy::State & gz_state, simulation_interfaces::msg::EntityState & state)
{
  // TODO(azeey) Fill in header
  state.pose.position.x = gz_state.pose.X();
  state.pose.position.y = gz_state.pose.Y();
  state.pose.position.z = gz_state.pose.Z();

  state.pose.orientation.x = gz_state.pose.Rot().X();
  state.pose.orientation.y = gz_state.pose.Rot().Y();
  state.pose.orientation.z = gz_state.pose.Rot().Z();
  state.pose.orientation.w = gz_state.pose.Rot().W();

  // TODO(azeey) Add support for twists and accelerations
  // TODO(azeey) Implement error checking and setting error message
  return true;
}

void ConvertPose(const gz::math::Pose3d & gz_pose, geometry_msgs::msg::Pose & ros_pose)
{
  ros_pose.position.x = gz_pose.X();
  ros_pose.position.y = gz_pose.Y();
  ros_pose.position.z = gz_pose.Z();

  ros_pose.orientation.x = gz_pose.Rot().X();
  ros_pose.orientation.y = gz_pose.Rot().Y();
  ros_pose.orientation.z = gz_pose.Rot().Z();
  ros_pose.orientation.w = gz_pose.Rot().W();
}

geometry_msgs::msg::Pose ConvertPose(const gz::math::Pose3d & gz_pose)
{
  geometry_msgs::msg::Pose ros_pose;
  ConvertPose(gz_pose, ros_pose);
  return ros_pose;
}

void ConvertPose(const geometry_msgs::msg::Pose & ros_pose, gz::math::Pose3d & gz_pose){
  gz_pose.Pos().X() = ros_pose.position.x;
  gz_pose.Pos().Y() = ros_pose.position.y;
  gz_pose.Pos().Z() = ros_pose.position.z;

  gz_pose.Rot().X() = ros_pose.orientation.x;
  gz_pose.Rot().Y() = ros_pose.orientation.y;
  gz_pose.Rot().Z() = ros_pose.orientation.z;
  gz_pose.Rot().W() = ros_pose.orientation.w;
}

gz::math::Pose3d  ConvertPose(const geometry_msgs::msg::Pose & ros_pose) {
  gz::math::Pose3d gz_pose;
  ConvertPose(ros_pose, gz_pose);
  return gz_pose;
}


void ConvertVector3(const geometry_msgs::msg::Vector3 & ros_v, gz::math::Vector3d & gz_v) {
  gz_v.X() = ros_v.x;
  gz_v.Y() = ros_v.y;
  gz_v.Z() = ros_v.z;
}

gz::math::Vector3d ConvertVector3(const geometry_msgs::msg::Vector3 & ros_v) {
  gz::math::Vector3d gz_v;
  ConvertVector3(ros_v, gz_v);
  return gz_v;
}

}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
