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

#include <memory>
#include <string>

// include builtin interfaces
#include "ros1_ign_bridge/builtin_interfaces_factories.hpp"
#include "ros1_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros1_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros1_type_name,
  const std::string & ign_type_name)
{
  // mapping from string to specialized template
  if (
    (ros1_type_name == "std_msgs/Float32" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Float")
  {
    return std::make_shared<
      Factory<
        std_msgs::Float32,
        ignition::msgs::Float
      >
    >("std_msgs/Float32", ign_type_name);
  }
  if (
    (ros1_type_name == "std_msgs/Header" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Header")
  {
    return std::make_shared<
      Factory<
        std_msgs::Header,
        ignition::msgs::Header
      >
    >("std_msgs/Header", ign_type_name);
  }
  if (
    (ros1_type_name == "std_msgs/String" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.StringMsg")
  {
    return std::make_shared<
      Factory<
        std_msgs::String,
        ignition::msgs::StringMsg
      >
    >("std_msgs/String", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/Quaternion" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Quaternion")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Quaternion,
        ignition::msgs::Quaternion
      >
    >("geometry_msgs/Quaternion", ign_type_name);
  }
  if (
    (ros1_type_name == "rosgraph_msgs/Clock" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Clock")
  {
    return std::make_shared<
      Factory<
        rosgraph_msgs::Clock,
        ignition::msgs::Clock
      >
    >("rosgraph_msgs/Clock", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/Vector3" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Vector3d")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Vector3,
        ignition::msgs::Vector3d
      >
    >("geometry_msgs/Vector3", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/Point" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Vector3d")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Point,
        ignition::msgs::Vector3d
      >
    >("geometry_msgs/Point", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/Pose" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Pose,
        ignition::msgs::Pose
      >
    >("geometry_msgs/Pose", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/PoseStamped" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::PoseStamped,
        ignition::msgs::Pose
      >
    >("geometry_msgs/PoseStamped", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/Transform" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Transform,
        ignition::msgs::Pose
      >
    >("geometry_msgs/Transform", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/TransformStamped" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::TransformStamped,
        ignition::msgs::Pose
      >
    >("geometry_msgs/TransformStamped", ign_type_name);
  }
  if (
    (ros1_type_name == "geometry_msgs/Twist" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Twist")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Twist,
        ignition::msgs::Twist
      >
    >("geometry_msgs/Twist", ign_type_name);
  }
  if (
    (ros1_type_name == "mav_msgs/Actuators" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Actuators")
  {
    return std::make_shared<
      Factory<
        mav_msgs::Actuators,
        ignition::msgs::Actuators
      >
    >("mav_msgs/Actuators", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/FluidPressure" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Fluid")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::FluidPressure,
        ignition::msgs::Fluid
      >
    >("sensor_msgs/FluidPressure", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/Image" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Image")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::Image,
        ignition::msgs::Image
      >
    >("sensor_msgs/Image", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/Imu" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.IMU")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::Imu,
        ignition::msgs::IMU
      >
    >("sensor_msgs/Imu", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/JointState" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Model")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::JointState,
        ignition::msgs::Model
      >
    >("sensor_msgs/JointState", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/LaserScan" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.LaserScan")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::LaserScan,
        ignition::msgs::LaserScan
      >
    >("sensor_msgs/LaserScan", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/MagneticField" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.Magnetometer")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::MagneticField,
        ignition::msgs::Magnetometer
      >
    >("sensor_msgs/Magnetometer", ign_type_name);
  }
  if (
    (ros1_type_name == "sensor_msgs/PointCloud2" || ros1_type_name == "") &&
     ign_type_name == "ignition.msgs.PointCloud")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::PointCloud2,
        ignition::msgs::PointCloud
      >
    >("sensor_msgs/PointCloud2", ign_type_name);
  }
  return std::shared_ptr<FactoryInterface>();
}

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros1_type_name,
            const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> factory;
  factory = get_factory_builtin_interfaces(ros1_type_name, ign_type_name);
  if (factory)
    return factory;

  throw std::runtime_error("No template specialization for the pair");
};

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_1_to_ign(
  const std_msgs::Float32 & ros1_msg,
  ignition::msgs::Float & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Float32,
  ignition::msgs::Float
>::convert_ign_to_1(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_1_to_ign(
  const std_msgs::Header & ros1_msg,
  ignition::msgs::Header & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Header,
  ignition::msgs::Header
>::convert_ign_to_1(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_1_to_ign(
  const std_msgs::String & ros1_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::String,
  ignition::msgs::StringMsg
>::convert_ign_to_1(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

// rosgraph_msgs
template<>
void
Factory<
  rosgraph_msgs::Clock,
  ignition::msgs::Clock
>::convert_1_to_ign(
  const rosgraph_msgs::Clock & ros1_msg,
  ignition::msgs::Clock & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  rosgraph_msgs::Clock,
  ignition::msgs::Clock
>::convert_ign_to_1(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

// geometry_msgs
template<>
void
Factory<
  geometry_msgs::Quaternion,
  ignition::msgs::Quaternion
>::convert_1_to_ign(
  const geometry_msgs::Quaternion & ros1_msg,
  ignition::msgs::Quaternion & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Quaternion,
  ignition::msgs::Quaternion
>::convert_ign_to_1(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::Vector3,
  ignition::msgs::Vector3d
>::convert_1_to_ign(
  const geometry_msgs::Vector3 & ros1_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Vector3,
  ignition::msgs::Vector3d
>::convert_ign_to_1(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::Point,
  ignition::msgs::Vector3d
>::convert_1_to_ign(
  const geometry_msgs::Point & ros1_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Point,
  ignition::msgs::Vector3d
>::convert_ign_to_1(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::Pose,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::Pose & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Pose,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::PoseStamped & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::Transform,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::Transform & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Transform,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  ignition::msgs::Pose
>::convert_1_to_ign(
  const geometry_msgs::TransformStamped & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  ignition::msgs::Pose
>::convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  geometry_msgs::Twist,
  ignition::msgs::Twist
>::convert_1_to_ign(
  const geometry_msgs::Twist & ros1_msg,
  ignition::msgs::Twist & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Twist,
  ignition::msgs::Twist
>::convert_ign_to_1(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

// mav_msgs
template<>
void
Factory<
  mav_msgs::Actuators,
  ignition::msgs::Actuators
>::convert_1_to_ign(
  const mav_msgs::Actuators & ros1_msg,
  ignition::msgs::Actuators & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  mav_msgs::Actuators,
  ignition::msgs::Actuators
>::convert_ign_to_1(
  const ignition::msgs::Actuators & ign_msg,
  mav_msgs::Actuators & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

// sensor_msgs
template<>
void
Factory<
  sensor_msgs::FluidPressure,
  ignition::msgs::Fluid
>::convert_1_to_ign(
  const sensor_msgs::FluidPressure & ros1_msg,
  ignition::msgs::Fluid & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::FluidPressure,
  ignition::msgs::Fluid
>::convert_ign_to_1(
  const ignition::msgs::Fluid & ign_msg,
  sensor_msgs::FluidPressure & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  sensor_msgs::Image,
  ignition::msgs::Image
>::convert_1_to_ign(
  const sensor_msgs::Image & ros1_msg,
  ignition::msgs::Image & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::Image,
  ignition::msgs::Image
>::convert_ign_to_1(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::Image & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  sensor_msgs::Imu,
  ignition::msgs::IMU
>::convert_1_to_ign(
  const sensor_msgs::Imu & ros1_msg,
  ignition::msgs::IMU & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::Imu,
  ignition::msgs::IMU
>::convert_ign_to_1(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  sensor_msgs::JointState,
  ignition::msgs::Model
>::convert_1_to_ign(
  const sensor_msgs::JointState & ros1_msg,
  ignition::msgs::Model & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::JointState,
  ignition::msgs::Model
>::convert_ign_to_1(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::JointState & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  sensor_msgs::LaserScan,
  ignition::msgs::LaserScan
>::convert_1_to_ign(
  const sensor_msgs::LaserScan & ros1_msg,
  ignition::msgs::LaserScan & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::LaserScan,
  ignition::msgs::LaserScan
>::convert_ign_to_1(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  sensor_msgs::MagneticField,
  ignition::msgs::Magnetometer
>::convert_1_to_ign(
  const sensor_msgs::MagneticField & ros1_msg,
  ignition::msgs::Magnetometer & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::MagneticField,
  ignition::msgs::Magnetometer
>::convert_ign_to_1(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  ignition::msgs::PointCloud
>::convert_1_to_ign(
  const sensor_msgs::PointCloud2 & ros1_msg,
  ignition::msgs::PointCloud & ign_msg)
{
  ros1_ign_bridge::convert_1_to_ign(ros1_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  ignition::msgs::PointCloud
>::convert_ign_to_1(
  const ignition::msgs::PointCloud & ign_msg,
  sensor_msgs::PointCloud2 & ros1_msg)
{
  ros1_ign_bridge::convert_ign_to_1(ign_msg, ros1_msg);
}

}  // namespace ros1_ign_bridge
