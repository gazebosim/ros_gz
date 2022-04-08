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

#include "factories/ros_ign_interfaces.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_ign_bridge/convert/ros_ign_interfaces.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__ros_ign_interfaces(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  if (
    (ros_type_name == "ros_ign_interfaces/msg/JointWrench" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.JointWrench")
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::JointWrench,
        ignition::msgs::JointWrench
      >
    >("ros_ign_interfaces/msg/JointWrench", ign_type_name);
  }
  if (
    (ros_type_name == "ros_ign_interfaces/msg/Entity" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Entity")
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Entity,
        ignition::msgs::Entity
      >
    >("ros_ign_interfaces/msg/Entity", ign_type_name);
  }
  if (
    (ros_type_name == "ros_ign_interfaces/msg/Contact" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Contact")
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Contact,
        ignition::msgs::Contact
      >
    >("ros_ign_interfaces/msg/Contact", ign_type_name);
  }
  if (
    (ros_type_name == "ros_ign_interfaces/msg/Contacts" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Contacts")
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Contacts,
        ignition::msgs::Contacts
      >
    >("ros_ign_interfaces/msg/Contacts", ign_type_name);
  }
  if (
    (ros_type_name == "ros_ign_interfaces/msg/Dataframe" || ros_type_name.empty()) &&
    ign_type_name == "ignition.msgs.Dataframe")
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Dataframe,
        ignition::msgs::Dataframe
      >
    >("ros_ign_interfaces/msg/Dataframe", ign_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/Light" ||
    ros_type_name.empty()) && ign_type_name == "ignition.msgs.Light")
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Light,
        ignition::msgs::Light
      >
    >("ros_ign_interfaces/msg/Light", ign_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  ros_ign_interfaces::msg::JointWrench,
  ignition::msgs::JointWrench
>::convert_ros_to_ign(
  const ros_ign_interfaces::msg::JointWrench & ros_msg,
  ignition::msgs::JointWrench & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::JointWrench,
  ignition::msgs::JointWrench
>::convert_ign_to_ros(
  const ignition::msgs::JointWrench & ign_msg,
  ros_ign_interfaces::msg::JointWrench & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Entity,
  ignition::msgs::Entity
>::convert_ros_to_ign(
  const ros_ign_interfaces::msg::Entity & ros_msg,
  ignition::msgs::Entity & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Entity,
  ignition::msgs::Entity
>::convert_ign_to_ros(
  const ignition::msgs::Entity & ign_msg,
  ros_ign_interfaces::msg::Entity & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contact,
  ignition::msgs::Contact
>::convert_ros_to_ign(
  const ros_ign_interfaces::msg::Contact & ros_msg,
  ignition::msgs::Contact & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contact,
  ignition::msgs::Contact
>::convert_ign_to_ros(
  const ignition::msgs::Contact & ign_msg,
  ros_ign_interfaces::msg::Contact & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contacts,
  ignition::msgs::Contacts
>::convert_ros_to_ign(
  const ros_ign_interfaces::msg::Contacts & ros_msg,
  ignition::msgs::Contacts & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contacts,
  ignition::msgs::Contacts
>::convert_ign_to_ros(
  const ignition::msgs::Contacts & ign_msg,
  ros_ign_interfaces::msg::Contacts & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Dataframe,
  ignition::msgs::Dataframe
>::convert_ros_to_ign(
  const ros_ign_interfaces::msg::Dataframe & ros_msg,
  ignition::msgs::Dataframe & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Dataframe,
  ignition::msgs::Dataframe
>::convert_ign_to_ros(
  const ignition::msgs::Dataframe & ign_msg,
  ros_ign_interfaces::msg::Dataframe & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Light,
  ignition::msgs::Light
>::convert_ros_to_ign(
  const ros_ign_interfaces::msg::Light & ros_msg,
  ignition::msgs::Light & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Light,
  ignition::msgs::Light
>::convert_ign_to_ros(
  const ignition::msgs::Light & ign_msg,
  ros_ign_interfaces::msg::Light & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
