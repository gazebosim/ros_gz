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

#include "factories/geometry_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/geometry_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__geometry_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "geometry_msgs/msg/Quaternion" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Quaternion" ||
    gz_type_name == "ignition.msgs.Quaternion"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Quaternion,
        ignition::msgs::Quaternion
      >
    >("geometry_msgs/msg/Quaternion", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/Vector3" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Vector3d" ||
    gz_type_name == "ignition.msgs.Vector3d"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Vector3,
        ignition::msgs::Vector3d
      >
    >("geometry_msgs/msg/Vector3", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/Point" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Vector3d" ||
    gz_type_name == "ignition.msgs.Vector3d"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Point,
        ignition::msgs::Vector3d
      >
    >("geometry_msgs/msg/Point", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/Pose" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Pose" ||
    gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Pose,
        ignition::msgs::Pose
      >
    >("geometry_msgs/msg/Pose", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/PoseWithCovariance" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.PoseWithCovariance" ||
    gz_type_name == "ignition.msgs.PoseWithCovariance"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::PoseWithCovariance,
        ignition::msgs::PoseWithCovariance
      >
    >("geometry_msgs/msg/PoseWithCovariance", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/PoseStamped" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Pose" ||
    gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::PoseStamped,
        ignition::msgs::Pose
      >
    >("geometry_msgs/msg/PoseStamped", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/Transform" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Pose" ||
    gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Transform,
        ignition::msgs::Pose
      >
    >("geometry_msgs/msg/Transform", gz_type_name);
  }
  if ((ros_type_name == "geometry_msgs/msg/TransformStamped" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Pose" ||
    gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::TransformStamped,
        ignition::msgs::Pose
      >
    >("geometry_msgs/msg/TransformStamped", gz_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/msg/Twist" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Twist" ||
    gz_type_name == "ignition.msgs.Twist"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Twist,
        ignition::msgs::Twist
      >
    >("geometry_msgs/msg/Twist", gz_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/msg/TwistWithCovariance" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.TwistWithCovariance" ||
    gz_type_name == "ignition.msgs.TwistWithCovariance"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::TwistWithCovariance,
        ignition::msgs::TwistWithCovariance
      >
    >("geometry_msgs/msg/TwistWithCovariance", gz_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/msg/Wrench" || ros_type_name.empty()) &&
    (gz_type_name == "gz.msgs.Wrench" ||
    gz_type_name == "ignition.msgs.Wrench"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Wrench,
        ignition::msgs::Wrench
      >
    >("geometry_msgs/msg/Wrench", gz_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  geometry_msgs::msg::Quaternion,
  ignition::msgs::Quaternion
>::convert_ros_to_gz(
  const geometry_msgs::msg::Quaternion & ros_msg,
  ignition::msgs::Quaternion & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Quaternion,
  ignition::msgs::Quaternion
>::convert_gz_to_ros(
  const ignition::msgs::Quaternion & gz_msg,
  geometry_msgs::msg::Quaternion & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Vector3,
  ignition::msgs::Vector3d
>::convert_ros_to_gz(
  const geometry_msgs::msg::Vector3 & ros_msg,
  ignition::msgs::Vector3d & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Vector3,
  ignition::msgs::Vector3d
>::convert_gz_to_ros(
  const ignition::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Vector3 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Point,
  ignition::msgs::Vector3d
>::convert_ros_to_gz(
  const geometry_msgs::msg::Point & ros_msg,
  ignition::msgs::Vector3d & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Point,
  ignition::msgs::Vector3d
>::convert_gz_to_ros(
  const ignition::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Point & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Pose,
  ignition::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::Pose & ros_msg,
  ignition::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Pose,
  ignition::msgs::Pose
>::convert_gz_to_ros(
  const ignition::msgs::Pose & gz_msg,
  geometry_msgs::msg::Pose & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseWithCovariance,
  ignition::msgs::PoseWithCovariance
>::convert_ros_to_gz(
  const geometry_msgs::msg::PoseWithCovariance & ros_msg,
  ignition::msgs::PoseWithCovariance & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseWithCovariance,
  ignition::msgs::PoseWithCovariance
>::convert_gz_to_ros(
  const ignition::msgs::PoseWithCovariance & gz_msg,
  geometry_msgs::msg::PoseWithCovariance & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseStamped,
  ignition::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::PoseStamped & ros_msg,
  ignition::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseStamped,
  ignition::msgs::Pose
>::convert_gz_to_ros(
  const ignition::msgs::Pose & gz_msg,
  geometry_msgs::msg::PoseStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Transform,
  ignition::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::Transform & ros_msg,
  ignition::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Transform,
  ignition::msgs::Pose
>::convert_gz_to_ros(
  const ignition::msgs::Pose & gz_msg,
  geometry_msgs::msg::Transform & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TransformStamped,
  ignition::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::TransformStamped & ros_msg,
  ignition::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TransformStamped,
  ignition::msgs::Pose
>::convert_gz_to_ros(
  const ignition::msgs::Pose & gz_msg,
  geometry_msgs::msg::TransformStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Twist,
  ignition::msgs::Twist
>::convert_ros_to_gz(
  const geometry_msgs::msg::Twist & ros_msg,
  ignition::msgs::Twist & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Twist,
  ignition::msgs::Twist
>::convert_gz_to_ros(
  const ignition::msgs::Twist & gz_msg,
  geometry_msgs::msg::Twist & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TwistWithCovariance,
  ignition::msgs::TwistWithCovariance
>::convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovariance & ros_msg,
  ignition::msgs::TwistWithCovariance & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TwistWithCovariance,
  ignition::msgs::TwistWithCovariance
>::convert_gz_to_ros(
  const ignition::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovariance & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Wrench,
  ignition::msgs::Wrench
>::convert_ros_to_gz(
  const geometry_msgs::msg::Wrench & ros_msg,
  ignition::msgs::Wrench & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Wrench,
  ignition::msgs::Wrench
>::convert_gz_to_ros(
  const ignition::msgs::Wrench & gz_msg,
  geometry_msgs::msg::Wrench & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
