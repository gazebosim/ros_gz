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

#ifndef ROS1_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
#define ROS1_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_

// include ROS 1 messages
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

// include Ignition Transport messages
#include <ignition/msgs.hh>

#include <memory>
#include <string>

#include "ros1_ign_bridge/factory.hpp"

namespace ros1_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros1_type_name,
  const std::string & ign_type_name);

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros1_type_name,
            const std::string & ign_type_name);

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_1_to_ign(
  const std_msgs::Header & ros1_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ign_to_1(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros1_msg);

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_1_to_ign(
  const std_msgs::String & ros1_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ign_to_1(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros1_msg);

// geometry_msgs
template<>
void
Factory<
  geometry_msgs::Quaternion,
  ignition::msgs::Quaternion
>::convert_1_to_ign(
  const geometry_msgs::Quaternion & ros1_msg,
  ignition::msgs::Quaternion & ign_msg);

template<>
void
Factory<
  geometry_msgs::Quaternion,
  ignition::msgs::Quaternion
>::convert_ign_to_1(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros1_msg);

template<>
void
Factory<
  geometry_msgs::Vector3,
  ignition::msgs::Vector3d
>::convert_1_to_ign(
  const geometry_msgs::Vector3 & ros1_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
Factory<
  geometry_msgs::Vector3,
  ignition::msgs::Vector3d
>::convert_ign_to_1(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros1_msg);

template<>
void
Factory<
  geometry_msgs::Point,
  ignition::msgs::Vector3d
>::convert_1_to_ign(
  const geometry_msgs::Point & ros1_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
Factory<
  geometry_msgs::Point,
  ignition::msgs::Vector3d
>::convert_ign_to_1(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros1_msg);

template<>
void
Factory<
  geometry_msgs::Pose,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::Pose & ros1_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::Pose,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros1_msg);

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::PoseStamped & ros1_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros1_msg);

template<>
void
Factory<
  geometry_msgs::Transform,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::Transform & ros1_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::Transform,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros1_msg);

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::TransformStamped & ros1_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros1_msg);

// sensor_msgs
template<>
void
Factory<
  sensor_msgs::FluidPressure,
  ignition::msgs::Fluid
>::convert_1_to_ign(
  const sensor_msgs::FluidPressure & ros1_msg,
  ignition::msgs::Fluid & ign_msg);

template<>
void
Factory<
  sensor_msgs::FluidPressure,
  ignition::msgs::Fluid
>::convert_ign_to_1(
  const ignition::msgs::Fluid & ign_msg,
  sensor_msgs::FluidPressure & ros1_msg);

template<>
void
Factory<
  sensor_msgs::Image,
  ignition::msgs::Image
>::convert_1_to_ign(
  const sensor_msgs::Image & ros1_msg,
  ignition::msgs::Image & ign_msg);

template<>
void
Factory<
  sensor_msgs::Image,
  ignition::msgs::Image
>::convert_ign_to_1(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::Image & ros1_msg);

template<>
void
Factory<
  sensor_msgs::Imu,
  ignition::msgs::IMU
>::convert_1_to_ign(
  const sensor_msgs::Imu & ros1_msg,
  ignition::msgs::IMU & ign_msg);

template<>
void
Factory<
  sensor_msgs::Imu,
  ignition::msgs::IMU
>::convert_ign_to_1(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros1_msg);

template<>
void
Factory<
  sensor_msgs::LaserScan,
  ignition::msgs::LaserScan
>::convert_1_to_ign(
  const sensor_msgs::LaserScan & ros1_msg,
  ignition::msgs::LaserScan & ign_msg);

template<>
void
Factory<
  sensor_msgs::LaserScan,
  ignition::msgs::LaserScan
>::convert_ign_to_1(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros1_msg);

template<>
void
Factory<
  sensor_msgs::MagneticField,
  ignition::msgs::Magnetometer
>::convert_1_to_ign(
  const sensor_msgs::MagneticField & ros1_msg,
  ignition::msgs::Magnetometer & ign_msg);

template<>
void
Factory<
  sensor_msgs::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ign_to_1(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros1_msg);

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  ignition::msgs::PointCloud
>::convert_1_to_ign(
  const sensor_msgs::PointCloud2 & ros1_msg,
  ignition::msgs::PointCloud & ign_msg);

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  ignition::msgs::PointCloud
>::convert_ign_to_1(
  const ignition::msgs::PointCloud & ign_msg,
  sensor_msgs::PointCloud2 & ros1_msg);

}  // namespace ros1_ign_bridge

#endif  // ROS1_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
