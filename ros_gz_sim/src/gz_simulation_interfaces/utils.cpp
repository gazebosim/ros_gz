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
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
