// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef ROS_IGN_BRIDGE__FACTORIES__SENSOR_MSGS_HPP_
#define ROS_IGN_BRIDGE__FACTORIES__SENSOR_MSGS_HPP_

// ROS messages
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>

// Ignition messages
#include <ignition/msgs.hh>

#include "factory.hpp"

#include <memory>
#include <string>

namespace ros_ign_bridge 
{
std::shared_ptr<FactoryInterface>
get_factory_sensor_msgs(
            const std::string & ros_type_name,
            const std::string & ign_type_name);

template<>
void
Factory<
  sensor_msgs::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ros_to_ign(
  const sensor_msgs::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & ign_msg);

template<>
void
Factory<
  sensor_msgs::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ign_to_ros(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::FluidPressure & ros_msg);

template<>
void
Factory<
  sensor_msgs::Image,
  ignition::msgs::Image
>::convert_ros_to_ign(
  const sensor_msgs::Image & ros_msg,
  ignition::msgs::Image & ign_msg);

template<>
void
Factory<
  sensor_msgs::Image,
  ignition::msgs::Image
>::convert_ign_to_ros(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::Image & ros_msg);

template<>
void
Factory<
  sensor_msgs::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ros_to_ign(
  const sensor_msgs::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & ign_msg);

template<>
void
Factory<
  sensor_msgs::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ign_to_ros(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::CameraInfo & ros_msg);

template<>
void
Factory<
  sensor_msgs::Imu,
  ignition::msgs::IMU
>::convert_ros_to_ign(
  const sensor_msgs::Imu & ros_msg,
  ignition::msgs::IMU & ign_msg);

template<>
void
Factory<
  sensor_msgs::Imu,
  ignition::msgs::IMU
>::convert_ign_to_ros(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros_msg);

template<>
void
Factory<
  sensor_msgs::JointState,
  ignition::msgs::Model
>::convert_ros_to_ign(
  const sensor_msgs::JointState & ros_msg,
  ignition::msgs::Model & ign_msg);

template<>
void
Factory<
  sensor_msgs::JointState,
  ignition::msgs::Model
>::convert_ign_to_ros(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::JointState & ros_msg);

template<>
void
Factory<
  sensor_msgs::LaserScan,
  ignition::msgs::LaserScan
>::convert_ros_to_ign(
  const sensor_msgs::LaserScan & ros_msg,
  ignition::msgs::LaserScan & ign_msg);

template<>
void
Factory<
  sensor_msgs::LaserScan,
  ignition::msgs::LaserScan
>::convert_ign_to_ros(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros_msg);

template<>
void
Factory<
  sensor_msgs::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ros_to_ign(
  const sensor_msgs::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & ign_msg);

template<>
void
Factory<
  sensor_msgs::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ign_to_ros(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros_msg);

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ros_to_ign(
  const sensor_msgs::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked & ign_msg);

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ign_to_ros(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::PointCloud2 & ros_msg);

template<>
void
Factory<
  sensor_msgs::BatteryState,
  ignition::msgs::BatteryState
>::convert_ros_to_ign(
  const sensor_msgs::BatteryState & ros_msg,
  ignition::msgs::BatteryState & ign_msg);

template<>
void
Factory<
  sensor_msgs::BatteryState,
  ignition::msgs::BatteryState
>::convert_ign_to_ros(
  const ignition::msgs::BatteryState & ign_msg,
  sensor_msgs::BatteryState & ros_msg);

}  // namespace ros_ign_bridge
#endif  // ROS_IGN_BRIDGE__FACTORIES__STD_MSGS_HPP_
