// Copyright 2026 Honu Robotics
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

#include <gz/msgs/dvl_velocity_tracking.pb.h>

#include <cmath>
#include <cstdint>
#include <gz/math/Vector3.hh>
#include <gz/msgs/convert/Vector3.hh>
#include <marine_acoustic_msgs/msg/dvl.hpp>
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

  if (ros_msg.dvl_type == marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON) {
    gz_msg.set_type(gz::msgs::DVLVelocityTracking::DVL_TYPE_PISTON);
  } else if (ros_msg.dvl_type == marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PHASED_ARRAY) {
    gz_msg.set_type(gz::msgs::DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY);
  } else {
    gz_msg.set_type(gz::msgs::DVLVelocityTracking::DVL_TYPE_UNSPECIFIED);
  }

  // Beams
  uint8_t beamId = 1u;
  for (auto i = 0; i < ros_msg.num_good_beams; ++i) {
    gz::msgs::DVLBeamState *beam = gz_msg.add_beams();
    beam->set_id(beamId++);

    beam->mutable_velocity()->set_reference(
      gz::msgs::DVLKinematicEstimate::DVL_REFERENCE_SHIP);

    auto beam_unit = ros_msg.beam_unit_vec[i];
    auto beam_velocity = ros_msg.beam_velocity[i];
    beam->mutable_velocity()->mutable_mean()->set_x(beam_unit.x * beam_velocity);
    beam->mutable_velocity()->mutable_mean()->set_y(beam_unit.y * beam_velocity);
    beam->mutable_velocity()->mutable_mean()->set_z(beam_unit.z * beam_velocity);

    // The ROS beam_velocity_covar is a scalar variance along the beam axis.
    // It cannot be accurately mapped to a 3x3 covariance without additional
    // beam geometry information, so the covariance is left empty (unknown).

    beam->mutable_range()->set_mean(ros_msg.range[i]);
    beam->mutable_range()->set_variance(ros_msg.range_covar[i]);

    beam->set_rssi(ros_msg.beam_quality[i]);
    // nsd not available in ROS.

    beam->set_locked(true);
  }

  // Velocity.
  // The ROS Dvl message does not specify a velocity reference frame.
  // Assuming ship-frame (body-frame) velocity, which is the DVL's native output.
  gz_msg.mutable_velocity()->set_reference(
    gz::msgs::DVLKinematicEstimate::DVL_REFERENCE_SHIP);
  convert_ros_to_gz(ros_msg.velocity, (*gz_msg.mutable_velocity()->mutable_mean()));
  for (auto i = 0; i < 9; ++i) {
    gz_msg.mutable_velocity()->add_covariance(ros_msg.velocity_covar[i]);
  }

  // Target.
  if (ros_msg.velocity_mode == marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM) {
    gz_msg.mutable_target()->set_type(gz::msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM);
  } else if (ros_msg.velocity_mode == marine_acoustic_msgs::msg::Dvl::DVL_MODE_WATER) {
    gz_msg.mutable_target()->set_type(gz::msgs::DVLTrackingTarget::DVL_TARGET_WATER_MASS);
  } else {
    gz_msg.mutable_target()->set_type(gz::msgs::DVLTrackingTarget::DVL_TARGET_UNSPECIFIED);
  }

  // Map altitude to target range when available.
  // For bottom tracking, altitude is the distance to the sea floor.
  if (ros_msg.altitude >= 0) {
    gz_msg.mutable_target()->mutable_range()->set_mean(ros_msg.altitude);
    // target.range.variance defaults to 0 (unknown) in protobuf.
  }
  // target.position is not available from the ROS message.

  // ROS Dvl message has no status field; defaults to 0 (OK) in protobuf.
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::DVLVelocityTracking & gz_msg,
  marine_acoustic_msgs::msg::Dvl & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  if (gz_msg.target().type() == gz::msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM) {
    ros_msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
  } else if (gz_msg.target().type() == gz::msgs::DVLTrackingTarget::DVL_TARGET_WATER_MASS) {
    ros_msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_WATER;
  } else {
    // ROS has no "unspecified" target concept; default to bottom tracking.
    ros_msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
  }

  if (gz_msg.type() == gz::msgs::DVLVelocityTracking::DVL_TYPE_PISTON) {
    ros_msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;
  } else if (gz_msg.type() == gz::msgs::DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY) {
    ros_msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PHASED_ARRAY;
  } else {
    // ROS has no "unspecified" DVL type; default to piston.
    ros_msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;
  }

  // Note: The Gazebo velocity may be in DVL_REFERENCE_EARTH or DVL_REFERENCE_SHIP frame.
  // The ROS Dvl message does not specify a reference frame convention.
  // This bridge assumes both sides use the same frame convention (typically body-frame).
  convert_gz_to_ros(gz_msg.velocity().mean(), ros_msg.velocity);

  for (auto i = 0; i < 9; ++i) {
    if (gz_msg.velocity().covariance_size() > i) {
      ros_msg.velocity_covar[i] = gz_msg.velocity().covariance()[i];
    } else {
      ros_msg.velocity_covar[i] = -1;
    }
  }

  // Map altitude from target range when bottom tracking.
  if (gz_msg.target().type() == gz::msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM &&
    gz_msg.target().has_range() && gz_msg.target().range().mean() > 0)
  {
    ros_msg.altitude = gz_msg.target().range().mean();
  } else {
    ros_msg.altitude = -1;
  }
  ros_msg.course_gnd = std::atan2(ros_msg.velocity.y, ros_msg.velocity.x);
  ros_msg.speed_gnd = std::sqrt(ros_msg.velocity.x * ros_msg.velocity.x + ros_msg.velocity.y *
      ros_msg.velocity.y);

  // Unsupported in Gazebo.
  ros_msg.sound_speed = -1;

  ros_msg.beam_ranges_valid = false;
  ros_msg.beam_velocities_valid = false;

  // Crop num beams if needed.
  uint8_t numGoodBeams = 0u;
  auto numBeams = std::min(gz_msg.beams_size(), 4);

  for (auto i = 0; i < numBeams; ++i) {
    if (!gz_msg.beams()[i].locked()) {
      continue;
    }

    ros_msg.beam_ranges_valid = true;
    ros_msg.beam_velocities_valid = true;

    // Compute beam velocity vector once for reuse.
    gz::math::Vector3d v = gz::msgs::Convert(gz_msg.beams()[i].velocity().mean());
    const double vLen = v.Length();

    // Derive beam unit vector from the normalized beam velocity direction.
    if (vLen > 0) {
      gz::math::Vector3d unit = v / vLen;
      ros_msg.beam_unit_vec[numGoodBeams].x = unit.X();
      ros_msg.beam_unit_vec[numGoodBeams].y = unit.Y();
      ros_msg.beam_unit_vec[numGoodBeams].z = unit.Z();
    } else {
      ros_msg.beam_unit_vec[numGoodBeams].x = 0;
      ros_msg.beam_unit_vec[numGoodBeams].y = 0;
      ros_msg.beam_unit_vec[numGoodBeams].z = 0;
    }

    ros_msg.range[numGoodBeams] = gz_msg.beams()[i].range().mean();
    ros_msg.range_covar[numGoodBeams] = gz_msg.beams()[i].range().variance();
    ros_msg.beam_quality[numGoodBeams] = gz_msg.beams()[i].rssi();
    ros_msg.beam_velocity[numGoodBeams] = vLen;
    ros_msg.beam_velocity_covar[numGoodBeams] = -1;

    ++numGoodBeams;
  }

  ros_msg.num_good_beams = numGoodBeams;
}

}  // namespace ros_gz_bridge
