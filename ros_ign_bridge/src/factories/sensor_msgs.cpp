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

#include "factories/sensor_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_ign_bridge/convert/sensor_msgs.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__sensor_msgs(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if ((ros_type_name == "sensor_msgs/msg/FluidPressure" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.FluidPressure")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::FluidPressure,
        ignition::msgs::FluidPressure
      >
    >("sensor_msgs/msg/FluidPressure", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/Image" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Image")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Image,
        ignition::msgs::Image
      >
    >("sensor_msgs/msg/Image", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/CameraInfo" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.CameraInfo")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::CameraInfo,
        ignition::msgs::CameraInfo
      >
    >("sensor_msgs/msg/CameraInfo", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/Imu" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.IMU")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Imu,
        ignition::msgs::IMU
      >
    >("sensor_msgs/msg/Imu", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/JointState" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Model")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::JointState,
        ignition::msgs::Model
      >
    >("sensor_msgs/msg/JointState", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/LaserScan" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.LaserScan")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::LaserScan,
        ignition::msgs::LaserScan
      >
    >("sensor_msgs/msg/LaserScan", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/MagneticField" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Magnetometer")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::MagneticField,
        ignition::msgs::Magnetometer
      >
    >("sensor_msgs/msg/Magnetometer", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/PointCloud2" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.PointCloudPacked")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::PointCloud2,
        ignition::msgs::PointCloudPacked
      >
    >("sensor_msgs/msg/PointCloud2", ign_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/BatteryState" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.BatteryState")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::BatteryState,
        ignition::msgs::BatteryState
      >
    >("sensor_msgs/msg/BatteryState", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ros_to_ign(
  const sensor_msgs::msg::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ign_to_ros(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::msg::FluidPressure & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Image,
  ignition::msgs::Image
>::convert_ros_to_ign(
  const sensor_msgs::msg::Image & ros_msg,
  ignition::msgs::Image & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Image,
  ignition::msgs::Image
>::convert_ign_to_ros(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::msg::Image & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ros_to_ign(
  const sensor_msgs::msg::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ign_to_ros(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::msg::CameraInfo & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  ignition::msgs::IMU
>::convert_ros_to_ign(
  const sensor_msgs::msg::Imu & ros_msg,
  ignition::msgs::IMU & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  ignition::msgs::IMU
>::convert_ign_to_ros(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::msg::Imu & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  ignition::msgs::Model
>::convert_ros_to_ign(
  const sensor_msgs::msg::JointState & ros_msg,
  ignition::msgs::Model & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  ignition::msgs::Model
>::convert_ign_to_ros(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::msg::JointState & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  ignition::msgs::LaserScan
>::convert_ros_to_ign(
  const sensor_msgs::msg::LaserScan & ros_msg,
  ignition::msgs::LaserScan & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  ignition::msgs::LaserScan
>::convert_ign_to_ros(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::msg::LaserScan & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ros_to_ign(
  const sensor_msgs::msg::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ign_to_ros(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::msg::MagneticField & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ros_to_ign(
  const sensor_msgs::msg::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ign_to_ros(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  ignition::msgs::BatteryState
>::convert_ros_to_ign(
  const sensor_msgs::msg::BatteryState & ros_msg,
  ignition::msgs::BatteryState & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  ignition::msgs::BatteryState
>::convert_ign_to_ros(
  const ignition::msgs::BatteryState & ign_msg,
  sensor_msgs::msg::BatteryState & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
