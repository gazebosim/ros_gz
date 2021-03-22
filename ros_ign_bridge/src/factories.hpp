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

#ifndef ROS_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
#define ROS_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_

// ROS messages
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
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
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Ignition messages
#include <ignition/msgs.hh>

#include "factory.hpp"

#include <memory>
#include <string>

namespace ros_ign_bridge
{
std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros_type_name,
            const std::string & ign_type_name);

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::Bool,
  ignition::msgs::Boolean
>::convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg);

template<>
void
Factory<
  std_msgs::Bool,
  ignition::msgs::Boolean
>::convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg);

template<>
void
Factory<
  std_msgs::ColorRGBA,
  ignition::msgs::Color
>::convert_ros_to_ign(
  const std_msgs::ColorRGBA & ros_msg,
  ignition::msgs::Color & ign_msg);

template<>
void
Factory<
  std_msgs::ColorRGBA,
  ignition::msgs::Color
>::convert_ign_to_ros(
  const ignition::msgs::Color & ign_msg,
  std_msgs::ColorRGBA & ros_msg);

template<>
void
Factory<
  std_msgs::Empty,
  ignition::msgs::Empty
>::convert_ros_to_ign(
  const std_msgs::Empty & ros_msg,
  ignition::msgs::Empty & ign_msg);

template<>
void
Factory<
  std_msgs::Empty,
  ignition::msgs::Empty
>::convert_ign_to_ros(
  const ignition::msgs::Empty & ign_msg,
  std_msgs::Empty & ros_msg);

template<>
void
Factory<
  std_msgs::Int32,
  ignition::msgs::Int32
>::convert_ros_to_ign(
  const std_msgs::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg);

template<>
void
Factory<
  std_msgs::Int32,
  ignition::msgs::Int32
>::convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::Int32 & ros_msg);

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg);

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg);

template<>
void
Factory<
  std_msgs::Float64,
  ignition::msgs::Double
>::convert_ros_to_ign(
  const std_msgs::Float64 & ros_msg,
  ignition::msgs::Double & ign_msg);

template<>
void
Factory<
  std_msgs::Float64,
  ignition::msgs::Double
>::convert_ign_to_ros(
  const ignition::msgs::Double & ign_msg,
  std_msgs::Float64 & ros_msg);

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg);

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg);

// rosgraph_msgs
template<>
void
Factory<
  rosgraph_msgs::Clock,
  ignition::msgs::Clock
>::convert_ros_to_ign(
  const rosgraph_msgs::Clock & ros_msg,
  ignition::msgs::Clock & ign_msg);

template<>
void
Factory<
  rosgraph_msgs::Clock,
  ignition::msgs::Clock
>::convert_ign_to_ros(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros_msg);

// geometry_msgs
template<>
void
Factory<
  geometry_msgs::Quaternion,
  ignition::msgs::Quaternion
>::convert_ros_to_ign(
  const geometry_msgs::Quaternion & ros_msg,
  ignition::msgs::Quaternion & ign_msg);

template<>
void
Factory<
  geometry_msgs::Quaternion,
  ignition::msgs::Quaternion
>::convert_ign_to_ros(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros_msg);

template<>
void
Factory<
  geometry_msgs::Vector3,
  ignition::msgs::Vector3d
>::convert_ros_to_ign(
  const geometry_msgs::Vector3 & ros_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
Factory<
  geometry_msgs::Vector3,
  ignition::msgs::Vector3d
>::convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros_msg);

template<>
void
Factory<
  geometry_msgs::Point,
  ignition::msgs::Vector3d
>::convert_ros_to_ign(
  const geometry_msgs::Point & ros_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
Factory<
  geometry_msgs::Point,
  ignition::msgs::Vector3d
>::convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros_msg);

template<>
void
Factory<
  geometry_msgs::Pose,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::Pose,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros_msg);

template<>
void
Factory<
  geometry_msgs::PoseArray,
  ignition::msgs::Pose_V
>::convert_ros_to_ign(
  const geometry_msgs::PoseArray & ros_msg,
  ignition::msgs::Pose_V & ign_msg);

template<>
void
Factory<
  geometry_msgs::PoseArray,
  ignition::msgs::Pose_V
>::convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  geometry_msgs::PoseArray & ros_msg);

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros_msg);

template<>
void
Factory<
  geometry_msgs::Transform,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::Transform,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros_msg);

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  ignition::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::TransformStamped & ros_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  ignition::msgs::Pose
>::convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros_msg);

template<>
void
Factory<
  tf2_msgs::TFMessage,
  ignition::msgs::Pose_V
>::convert_ros_to_ign(
  const tf2_msgs::TFMessage & ros_msg,
  ignition::msgs::Pose_V & ign_msg);

template<>
void
Factory<
  tf2_msgs::TFMessage,
  ignition::msgs::Pose_V
>::convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  tf2_msgs::TFMessage & ros_msg);

template<>
void
Factory<
  geometry_msgs::Twist,
  ignition::msgs::Twist
>::convert_ros_to_ign(
  const geometry_msgs::Twist & ros_msg,
  ignition::msgs::Twist & ign_msg);

template<>
void
Factory<
  geometry_msgs::Twist,
  ignition::msgs::Twist
>::convert_ign_to_ros(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros_msg);

// mav_msgs
template<>
void
Factory<
  mav_msgs::Actuators,
  ignition::msgs::Actuators
>::convert_ros_to_ign(
  const mav_msgs::Actuators & ros_msg,
  ignition::msgs::Actuators & ign_msg);

template<>
void
Factory<
  mav_msgs::Actuators,
  ignition::msgs::Actuators
>::convert_ign_to_ros(
  const ignition::msgs::Actuators & ign_msg,
  mav_msgs::Actuators & ros_msg);

// nav_msgs
template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  ignition::msgs::OccupancyGrid
>::convert_ros_to_ign(
  const nav_msgs::OccupancyGrid & ros_msg,
  ignition::msgs::OccupancyGrid & ign_msg);

template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  ignition::msgs::OccupancyGrid
>::convert_ign_to_ros(
  const ignition::msgs::OccupancyGrid & ign_msg,
  nav_msgs::OccupancyGrid & ros_msg);

template<>
void
Factory<
  nav_msgs::Odometry,
  ignition::msgs::Odometry
>::convert_ros_to_ign(
  const nav_msgs::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg);

template<>
void
Factory<
  nav_msgs::Odometry,
  ignition::msgs::Odometry
>::convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros_msg);

// sensor_msgs
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

template<>
void
Factory<
  visualization_msgs::Marker,
  ignition::msgs::Marker
>::convert_ros_to_ign(
    const visualization_msgs::Marker & ros_msg,
    ignition::msgs::Marker & ign_msg);

template<>
void
Factory<
  visualization_msgs::Marker,
  ignition::msgs::Marker
>::convert_ign_to_ros(
    const ignition::msgs::Marker & ign_msg,
    visualization_msgs::Marker & ros_msg);

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  ignition::msgs::Marker_V
>::convert_ros_to_ign(
    const visualization_msgs::MarkerArray & ros_msg,
    ignition::msgs::Marker_V & ign_msg);

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  ignition::msgs::Marker_V
>::convert_ign_to_ros(
    const ignition::msgs::Marker_V & ign_msg,
    visualization_msgs::MarkerArray & ros_msg);

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
