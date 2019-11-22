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

#ifndef ROS_IGN_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_
#define ROS_IGN_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_

// include ROS builtin messages
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

// include Ignition builtin messages
#include <ignition/msgs.hh>

#include "ros_ign_bridge/convert_decl.hpp"

namespace ros_ign_bridge
{

// std_msgs
template<>
void
convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::Empty & ros_msg,
  ignition::msgs::Empty & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Empty & ign_msg,
  std_msgs::Empty & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg);

template<>
void
convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg);

// rosgraph_msgs
template<>
void
convert_ign_to_ros(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros_msg);

template<>
void
convert_ros_to_ign(
  const rosgraph_msgs::Clock & ros_msg,
  ignition::msgs::Clock & ign_msg);

// geometry_msgs
template<>
void
convert_ros_to_ign(
  const geometry_msgs::Quaternion & ros_msg,
  ignition::msgs::Quaternion & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Vector3 & ros_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Point & ros_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::TransformStamped & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros_msg);

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Twist & ros_msg,
  ignition::msgs::Twist & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros_msg);

// mav_msgs
template<>
void
convert_ros_to_ign(
  const mav_msgs::Actuators & ros_msg,
  ignition::msgs::Actuators & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Actuators & ign_msg,
  mav_msgs::Actuators & ros_msg);

// nav_msgs
template<>
void
convert_ros_to_ign(
  const nav_msgs::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros_msg);

// sensor_msgs
template<>
void
convert_ros_to_ign(
  const sensor_msgs::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::FluidPressure & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::Image & ros_msg,
  ignition::msgs::Image & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::Image & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::CameraInfo & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::Imu & ros_msg,
  ignition::msgs::IMU & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::JointState & ros_msg,
  ignition::msgs::Model & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::JointState & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::LaserScan & ros_msg,
  ignition::msgs::LaserScan & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::PointCloud2 & ros_msg);

template<>
void
convert_ros_to_ign(
  const sensor_msgs::BatteryState & ros_msg,
  ignition::msgs::BatteryState & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::BatteryState & ign_msg,
  sensor_msgs::BatteryState & ros_msg);

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_
