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

#ifndef ROS_GZ_BRIDGE__CONVERT__SENSOR_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__SENSOR_MSGS_HPP_

// Gazebo Msgs
#include <ignition/msgs/battery_state.pb.h>
#include <ignition/msgs/camera_info.pb.h>
#include <ignition/msgs/fluid_pressure.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/magnetometer.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/pointcloud_packed.pb.h>

// ROS 2 messages
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ros_ign_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

// sensor_msgs
template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::FluidPressure & gz_msg,
  sensor_msgs::msg::FluidPressure & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::Image & ros_msg,
  ignition::msgs::Image & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Image & gz_msg,
  sensor_msgs::msg::Image & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::CameraInfo & gz_msg,
  sensor_msgs::msg::CameraInfo & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::Imu & ros_msg,
  ignition::msgs::IMU & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::IMU & gz_msg,
  sensor_msgs::msg::Imu & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::JointState & ros_msg,
  ignition::msgs::Model & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Model & gz_msg,
  sensor_msgs::msg::JointState & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::LaserScan & ros_msg,
  ignition::msgs::LaserScan & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::LaserScan & gz_msg,
  sensor_msgs::msg::LaserScan & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Magnetometer & gz_msg,
  sensor_msgs::msg::MagneticField & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg);

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::BatteryState & ros_msg,
  ignition::msgs::BatteryState & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::BatteryState & gz_msg,
  sensor_msgs::msg::BatteryState & ros_msg);

}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__SENSOR_MSGS_HPP_
