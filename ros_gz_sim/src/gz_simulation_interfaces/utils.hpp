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

#ifndef GZ_SIMULATION_INTERFACES__UTILS_HPP_
#define GZ_SIMULATION_INTERFACES__UTILS_HPP_

#include <gz/msgs/time.pb.h>

#include <gz/math/Pose3.hh>
#include <simulation_interfaces/msg/entity_state.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
/// \brief Convert a gz::math Pose to a geometry_msgs Pose
/// \param[in] gz_pose gz::math::Pose object
/// \param[out] ros_pose geometry_msgs::msg::Pose object to be filled in
void ConvertPose(const gz::math::Pose3d & gz_pose, geometry_msgs::msg::Pose & ros_pose);

/// \brief Convert a gz::math Pose to a geometry_msgs Pose
/// \param[in] gz_pose gz::math::Pose object
/// \return Converted geometry_msgs::msg::Pose object
geometry_msgs::msg::Pose ConvertPose(const gz::math::Pose3d & gz_pose);

/// \brief Convert a geometry_msgs Pose to a gz::math Pose
/// \param[in] ros_pose geometry_msgs Pose object
/// \param[out] gz_pose gz::math::Pose object to be filled in
void ConvertPose(const geometry_msgs::msg::Pose & ros_pose, gz::math::Pose3d & gz_pose);

/// \brief Convert a geometry_msgs Pose to a gz::math Pose
/// \param[in] ros_pose geometry_msgs Pose object
/// \return Converted gz::math::Pose object
gz::math::Pose3d ConvertPose(const geometry_msgs::msg::Pose & ros_pose);

/// \brief Convert a gz::math Vector3d to a geometry_msgs Vector3
/// \param[in] gz_v gz::math::Vector3d object
/// \param[out] ros_v geometry_msgs::msg::Vector3 object to be filled in
void ConvertVector3(const gz::math::Vector3d & gz_v, geometry_msgs::msg::Vector3 & ros_v);

/// \brief Convert a gz::math Vector3d to a geometry_msgs Vector3
/// \param[in] gz_v gz::math::Vector3d object
/// \return Converted geometry_msgs::msg::Vector3
geometry_msgs::msg::Vector3 ConvertVector3(const gz::math::Vector3d & gz_v);

/// \brief Convert a geometry_msgs Vector3 to a gz::math Vector3
/// \param[in] ros_v geometry_msgs Vector3 object
/// \param[out] gz_v gz::math::Vector3d object to be filled in
void ConvertVector3(const geometry_msgs::msg::Vector3 & ros_v, gz::math::Vector3d & gz_v);

/// \brief Convert a ROS Vector3 to a Gazebo Vector3
/// \param[in] ros_v geometry_msgs Vector3 object
/// \return Converted gz::math::Vector3d object
gz::math::Vector3d ConvertVector3(const geometry_msgs::msg::Vector3 & ros_v);

/// \brief Convert Gazebo Time to ROS time
/// \param[in] gz_time gz::msgs::Time time
/// \return Converted ROS time
builtin_interfaces::msg::Time ConvertTime(const gz::msgs::Time gz_time);
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif  // GZ_SIMULATION_INTERFACES__UTILS_HPP_
