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

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__sensor_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "sensor_msgs/msg/FluidPressure" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.FluidPressure")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::FluidPressure,
        ignition::msgs::FluidPressure
      >
    >("sensor_msgs/msg/FluidPressure", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/Image" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.Image")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Image,
        ignition::msgs::Image
      >
    >("sensor_msgs/msg/Image", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/CameraInfo" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.CameraInfo")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::CameraInfo,
        ignition::msgs::CameraInfo
      >
    >("sensor_msgs/msg/CameraInfo", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/Imu" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.IMU")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Imu,
        ignition::msgs::IMU
      >
    >("sensor_msgs/msg/Imu", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/JointState" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.Model")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::JointState,
        ignition::msgs::Model
      >
    >("sensor_msgs/msg/JointState", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/LaserScan" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.LaserScan")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::LaserScan,
        ignition::msgs::LaserScan
      >
    >("sensor_msgs/msg/LaserScan", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/MagneticField" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.Magnetometer")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::MagneticField,
        ignition::msgs::Magnetometer
      >
    >("sensor_msgs/msg/Magnetometer", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/PointCloud2" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.PointCloudPacked")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::PointCloud2,
        ignition::msgs::PointCloudPacked
      >
    >("sensor_msgs/msg/PointCloud2", gz_type_name);
  }
  if ((ros_type_name == "sensor_msgs/msg/BatteryState" || ros_type_name.empty()) &&
    gz_type_name == "ignition.msgs.BatteryState")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::BatteryState,
        ignition::msgs::BatteryState
      >
    >("sensor_msgs/msg/BatteryState", gz_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_ros_to_gz(
  const sensor_msgs::msg::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  ignition::msgs::FluidPressure
>::convert_gz_to_ros(
  const ignition::msgs::FluidPressure & gz_msg,
  sensor_msgs::msg::FluidPressure & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Image,
  ignition::msgs::Image
>::convert_ros_to_gz(
  const sensor_msgs::msg::Image & ros_msg,
  ignition::msgs::Image & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Image,
  ignition::msgs::Image
>::convert_gz_to_ros(
  const ignition::msgs::Image & gz_msg,
  sensor_msgs::msg::Image & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_ros_to_gz(
  const sensor_msgs::msg::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  ignition::msgs::CameraInfo
>::convert_gz_to_ros(
  const ignition::msgs::CameraInfo & gz_msg,
  sensor_msgs::msg::CameraInfo & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  ignition::msgs::IMU
>::convert_ros_to_gz(
  const sensor_msgs::msg::Imu & ros_msg,
  ignition::msgs::IMU & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  ignition::msgs::IMU
>::convert_gz_to_ros(
  const ignition::msgs::IMU & gz_msg,
  sensor_msgs::msg::Imu & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  ignition::msgs::Model
>::convert_ros_to_gz(
  const sensor_msgs::msg::JointState & ros_msg,
  ignition::msgs::Model & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  ignition::msgs::Model
>::convert_gz_to_ros(
  const ignition::msgs::Model & gz_msg,
  sensor_msgs::msg::JointState & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  ignition::msgs::LaserScan
>::convert_ros_to_gz(
  const sensor_msgs::msg::LaserScan & ros_msg,
  ignition::msgs::LaserScan & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  ignition::msgs::LaserScan
>::convert_gz_to_ros(
  const ignition::msgs::LaserScan & gz_msg,
  sensor_msgs::msg::LaserScan & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ros_to_gz(
  const sensor_msgs::msg::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  ignition::msgs::Magnetometer
>::convert_gz_to_ros(
  const ignition::msgs::Magnetometer & gz_msg,
  sensor_msgs::msg::MagneticField & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_ros_to_gz(
  const sensor_msgs::msg::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  ignition::msgs::PointCloudPacked
>::convert_gz_to_ros(
  const ignition::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  ignition::msgs::BatteryState
>::convert_ros_to_gz(
  const sensor_msgs::msg::BatteryState & ros_msg,
  ignition::msgs::BatteryState & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  ignition::msgs::BatteryState
>::convert_gz_to_ros(
  const ignition::msgs::BatteryState & gz_msg,
  sensor_msgs::msg::BatteryState & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
