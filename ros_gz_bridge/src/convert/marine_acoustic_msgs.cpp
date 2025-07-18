// Copyright 2025 Honu Robotics
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

#include "gz/math/Vector3.hh"
#include "gz/msgs/convert/Vector3.hh"
#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/marine_acoustic_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const marine_acoustic_msgs::msg::Dvl & ros_msg,
  gz::msgs::DVLVelocityTracking & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  // for (auto i = 0u; i < ros_msg.position.size(); ++i) {
  //   gz_msg.add_position(ros_msg.position[i]);
  // }

  // for (auto i = 0u; i < ros_msg.velocity.size(); ++i) {
  //   gz_msg.add_velocity(ros_msg.velocity[i]);
  // }
  // for (auto i = 0u; i < ros_msg.normalized.size(); ++i) {
  //   gz_msg.add_normalized(ros_msg.normalized[i]);
  // }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::DVLVelocityTracking & gz_msg,
  marine_acoustic_msgs::msg::Dvl & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  if (gz_msg.target().type() == gz::msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM)
    ros_msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
  else if (gz_msg.target().type() == gz::msgs::DVLTrackingTarget::DVL_TARGET_WATER_MASS)
    ros_msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_WATER;

  if (gz_msg.type() == gz::msgs::DVLVelocityTracking::DVL_TYPE_PISTON)
    ros_msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;
  else if (gz_msg.type() == gz::msgs::DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY)
    ros_msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PHASED_ARRAY;

  convert_gz_to_ros(gz_msg.velocity().mean(), ros_msg.velocity);

  for (auto i = 0; i < 9; ++i)
    ros_msg.velocity_covar[i] = gz_msg.velocity().covariance()[i];

  // ros_msg.altitude =
  // ros_msg.corse_gnd =
  // ros_msg.speed_gnd =

  ros_msg.num_good_beams = gz_msg.beams_size();

  // Unsupported in Gazebo.
  ros_msg.sound_speed = 0;

  ros_msg.beam_ranges_valid = true;
  ros_msg.beam_velocities_valid = true;

  // Crop num beams if needed.
  auto numBeams = std::min(gz_msg.beams_size(), 4);

  for (auto i = 0; i < numBeams; ++i)
  {
    // ros_msg.beam_unit_vec = ;
    ros_msg.range[i] = gz_msg.beams()[i].range().mean();
    ros_msg.range_covar[i] = gz_msg.beams()[i].range().variance();
    ros_msg.beam_quality[i] = gz_msg.beams()[i].rssi();
    gz::math::Vector3d v = gz::msgs::Convert(gz_msg.beams()[i].velocity().mean());
    ros_msg.beam_velocity[i] = v.Length();
    ros_msg.beam_velocity_covar[i] = -1;
  }
}

}  // namespace ros_gz_bridge
