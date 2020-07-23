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

#ifndef FACTORIES_HPP_
#define FACTORIES_HPP_

// ROS 2 messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

// Ignition messages
#include <ignition/msgs.hh>

#include <memory>
#include <string>

#include "factory.hpp"

namespace ros_ign_bridge
{
std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros_type_name,
  const std::string & ign_type_name);

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::msg::Bool,
  ignition::msgs::Boolean
>::convert_ros_to_ign(
  const std_msgs::msg::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg);

template<>
void
Factory<
  std_msgs::msg::Bool,
  ignition::msgs::Boolean
>::convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::msg::Bool & ros_msg);

template<>
void
Factory<
  std_msgs::msg::Empty,
  ignition::msgs::Empty
>::convert_ros_to_ign(
  const std_msgs::msg::Empty & ros_msg,
  ignition::msgs::Empty & ign_msg);

template<>
void
Factory<
  std_msgs::msg::Empty,
  ignition::msgs::Empty
>::convert_ign_to_ros(
  const ignition::msgs::Empty & ign_msg,
  std_msgs::msg::Empty & ros_msg);

template<>
void
Factory<
  std_msgs::msg::Float32,
  ignition::msgs::Float
>::convert_ros_to_ign(
  const std_msgs::msg::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg);

template<>
void
Factory<
  std_msgs::msg::Float32,
  ignition::msgs::Float
>::convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::msg::Float32 & ros_msg);

template<>
void
Factory<
  std_msgs::msg::Header,
  ignition::msgs::Header
>::convert_ros_to_ign(
  const std_msgs::msg::Header & ros_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
Factory<
  std_msgs::msg::Header,
  ignition::msgs::Header
>::convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::msg::Header & ros_msg);

template<>
void
Factory<
  std_msgs::msg::Int32,
  ignition::msgs::Int32
>::convert_ros_to_ign(
  const std_msgs::msg::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg);

template<>
void
Factory<
  std_msgs::msg::Int32,
  ignition::msgs::Int32
>::convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::msg::Int32 & ros_msg);

template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ros_to_ign(
  const std_msgs::msg::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::msg::String & ros_msg);

// rosgraph_msgs
template<>
void
Factory<
  rosgraph_msgs::msg::Clock,
  ignition::msgs::Clock
>::convert_ros_to_ign(
  const rosgraph_msgs::msg::Clock & ros_msg,
  ignition::msgs::Clock & ign_msg);

template<>
void
Factory<
  rosgraph_msgs::msg::Clock,
  ignition::msgs::Clock
>::convert_ign_to_ros(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::msg::Clock & ros_msg);

// geometry_msgs
template<>
void
Factory<
  geometry_msgs::msg::Quaternion,
  ignition::msgs::Quaternion
>::convert_ros_to_ign(
  const geometry_msgs::msg::Quaternion & ros_msg,
  ignition::msgs::Quaternion & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::Quaternion,
  ignition::msgs::Quaternion
>::convert_ign_to_ros(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::msg::Quaternion & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::Vector3,
  ignition::msgs::Vector3d
>::convert_ros_to_ign(
  const geometry_msgs::msg::Vector3 & ros_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::Vector3,
  ignition::msgs::Vector3d
>::convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::msg::Vector3 & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::Point,
  ignition::msgs::Vector3d
>::convert_ros_to_ign(
  const geometry_msgs::msg::Point & ros_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::Point,
  ignition::msgs::Vector3d
>::convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::msg::Point & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::Pose,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::msg::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::Pose,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::Pose & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::PoseStamped,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::msg::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::PoseStamped,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::PoseStamped & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::Transform,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::msg::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::Transform,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::Transform & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::TransformStamped,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::msg::TransformStamped & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::TransformStamped,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::TransformStamped & ros_msg);

template<>
void
Factory<
  geometry_msgs::msg::Twist,
  ignition::msgs::Twist
>::convert_ros_to_ign(
  const geometry_msgs::msg::Twist & ros_msg,
  ignition::msgs::Twist & ign_msg);

template<>
void
Factory<
  geometry_msgs::msg::Twist,
  ignition::msgs::Twist
>::convert_ign_to_ros(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::msg::Twist & ros_msg);

// mav_msgs
/**
template<>
void
Factory<
  mav_msgs::msg::Actuators,
  ignition::msgs::Actuators
>::convert_ros_to_ign(
  const mav_msgs::msg::Actuators & ros_msg,
  ignition::msgs::Actuators & ign_msg);

template<>
void
Factory<
  mav_msgs::msg::Actuators,
  ignition::msgs::Actuators
>::convert_ign_to_ros(
  const ignition::msgs::Actuators & ign_msg,
  mav_msgs::msg::Actuators & ros_msg);
*/

// nav_msgs
template<>
void
Factory<
  nav_msgs::msg::Odometry,
  ignition::msgs::Odometry
>::convert_ros_to_ign(
  const nav_msgs::msg::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg);

template<>
void
Factory<
  nav_msgs::msg::Odometry,
  ignition::msgs::Odometry
>::convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::msg::Odometry & ros_msg);

// sensor_msgs
template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ros_to_ign(
  const sensor_msgs::msg::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ign_to_ros(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::msg::FluidPressure & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::Image,
  ignition::msgs::Image
>::convert_ros_to_ign(
  const sensor_msgs::msg::Image & ros_msg,
  ignition::msgs::Image & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::Image,
  ignition::msgs::Image
>::convert_ign_to_ros(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::msg::Image & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ros_to_ign(
  const sensor_msgs::msg::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ign_to_ros(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::msg::CameraInfo & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  ignition::msgs::IMU
>::convert_ros_to_ign(
  const sensor_msgs::msg::Imu & ros_msg,
  ignition::msgs::IMU & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  ignition::msgs::IMU
>::convert_ign_to_ros(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::msg::Imu & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  ignition::msgs::Model
>::convert_ros_to_ign(
  const sensor_msgs::msg::JointState & ros_msg,
  ignition::msgs::Model & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  ignition::msgs::Model
>::convert_ign_to_ros(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::msg::JointState & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  ignition::msgs::LaserScan
>::convert_ros_to_ign(
  const sensor_msgs::msg::LaserScan & ros_msg,
  ignition::msgs::LaserScan & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  ignition::msgs::LaserScan
>::convert_ign_to_ros(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::msg::LaserScan & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ros_to_ign(
  const sensor_msgs::msg::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ign_to_ros(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::msg::MagneticField & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ros_to_ign(
  const sensor_msgs::msg::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ign_to_ros(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg);

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  ignition::msgs::BatteryState
>::convert_ros_to_ign(
  const sensor_msgs::msg::BatteryState & ros_msg,
  ignition::msgs::BatteryState & ign_msg);

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  ignition::msgs::BatteryState
>::convert_ign_to_ros(
  const ignition::msgs::BatteryState & ign_msg,
  sensor_msgs::msg::BatteryState & ros_msg);

}  // namespace ros_ign_bridge

#endif  // FACTORIES_HPP_
