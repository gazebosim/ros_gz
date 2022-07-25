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

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__ros_ign_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "ros_ign_interfaces/msg/JointWrench" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.JointWrench" || gz_type_name == "ignition.msgs.JointWrench"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::JointWrench,
        ignition::msgs::JointWrench
      >
    >("ros_ign_interfaces/msg/JointWrench", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/Entity" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Entity" || gz_type_name == "ignition.msgs.Entity"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Entity,
        ignition::msgs::Entity
      >
    >("ros_ign_interfaces/msg/Entity", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/Contact" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Contact" || gz_type_name == "ignition.msgs.Contact"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Contact,
        ignition::msgs::Contact
      >
    >("ros_ign_interfaces/msg/Contact", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/Contacts" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Contacts" || gz_type_name == "ignition.msgs.Contacts"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Contacts,
        ignition::msgs::Contacts
      >
    >("ros_ign_interfaces/msg/Contacts", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/GuiCamera" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.GUICamera" || gz_type_name == "ignition.msgs.GUICamera"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::GuiCamera,
        ignition::msgs::GUICamera
      >
    >("ros_ign_interfaces/msg/GuiCamera", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/Light" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.Light" || gz_type_name == "ignition.msgs.Light"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::Light,
        ignition::msgs::Light
      >
    >("ros_ign_interfaces/msg/Light", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/StringVec" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.StringMsg_V" || gz_type_name == "ignition.msgs.StringMsg_V"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::StringVec,
        ignition::msgs::StringMsg_V
      >
    >("ros_ign_interfaces/msg/StringVec", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/TrackVisual" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.TrackVisual" || gz_type_name == "ignition.msgs.TrackVisual"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::TrackVisual,
        ignition::msgs::TrackVisual
      >
    >("ros_ign_interfaces/msg/TrackVisual", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/VideoRecord" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.VideoRecord" || gz_type_name == "ignition.msgs.VideoRecord"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::VideoRecord,
        ignition::msgs::VideoRecord
      >
    >("ros_ign_interfaces/msg/VideoRecord", gz_type_name);
  }
  if ((ros_type_name == "ros_ign_interfaces/msg/WorldControl" || ros_type_name.empty())
      && (gz_type_name == "gz.msgs.WorldControl" || gz_type_name == "ignition.msgs.WorldControl"))
  {
    return std::make_shared<
      Factory<
        ros_ign_interfaces::msg::WorldControl,
        ignition::msgs::WorldControl
      >
    >("ros_ign_interfaces/msg/WorldControl", gz_type_name);
  }
  return nullptr;
}

template<>
void
Factory<
  ros_ign_interfaces::msg::JointWrench,
  ignition::msgs::JointWrench
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::JointWrench & ros_msg,
  ignition::msgs::JointWrench & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::JointWrench,
  ignition::msgs::JointWrench
>::convert_gz_to_ros(
  const ignition::msgs::JointWrench & gz_msg,
  ros_ign_interfaces::msg::JointWrench & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Entity,
  ignition::msgs::Entity
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::Entity & ros_msg,
  ignition::msgs::Entity & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Entity,
  ignition::msgs::Entity
>::convert_gz_to_ros(
  const ignition::msgs::Entity & gz_msg,
  ros_ign_interfaces::msg::Entity & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contact,
  ignition::msgs::Contact
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::Contact & ros_msg,
  ignition::msgs::Contact & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contact,
  ignition::msgs::Contact
>::convert_gz_to_ros(
  const ignition::msgs::Contact & gz_msg,
  ros_ign_interfaces::msg::Contact & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contacts,
  ignition::msgs::Contacts
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::Contacts & ros_msg,
  ignition::msgs::Contacts & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Contacts,
  ignition::msgs::Contacts
>::convert_gz_to_ros(
  const ignition::msgs::Contacts & gz_msg,
  ros_ign_interfaces::msg::Contacts & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::GuiCamera,
  ignition::msgs::GUICamera
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::GuiCamera & ros_msg,
  ignition::msgs::GUICamera & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::GuiCamera,
  ignition::msgs::GUICamera
>::convert_gz_to_ros(
  const ignition::msgs::GUICamera & gz_msg,
  ros_ign_interfaces::msg::GuiCamera & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Light,
  ignition::msgs::Light
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::Light & ros_msg,
  ignition::msgs::Light & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::Light,
  ignition::msgs::Light
>::convert_gz_to_ros(
  const ignition::msgs::Light & gz_msg,
  ros_ign_interfaces::msg::Light & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::StringVec,
  ignition::msgs::StringMsg_V
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::StringVec & ros_msg,
  ignition::msgs::StringMsg_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::StringVec,
  ignition::msgs::StringMsg_V
>::convert_gz_to_ros(
  const ignition::msgs::StringMsg_V & gz_msg,
  ros_ign_interfaces::msg::StringVec & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::TrackVisual,
  ignition::msgs::TrackVisual
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::TrackVisual & ros_msg,
  ignition::msgs::TrackVisual & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::TrackVisual,
  ignition::msgs::TrackVisual
>::convert_gz_to_ros(
  const ignition::msgs::TrackVisual & gz_msg,
  ros_ign_interfaces::msg::TrackVisual & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::VideoRecord,
  ignition::msgs::VideoRecord
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::VideoRecord & ros_msg,
  ignition::msgs::VideoRecord & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::VideoRecord,
  ignition::msgs::VideoRecord
>::convert_gz_to_ros(
  const ignition::msgs::VideoRecord & gz_msg,
  ros_ign_interfaces::msg::VideoRecord & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::WorldControl,
  ignition::msgs::WorldControl
>::convert_ros_to_gz(
  const ros_ign_interfaces::msg::WorldControl & ros_msg,
  ignition::msgs::WorldControl & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_ign_interfaces::msg::WorldControl,
  ignition::msgs::WorldControl
>::convert_gz_to_ros(
  const ignition::msgs::WorldControl & gz_msg,
  ros_ign_interfaces::msg::WorldControl & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
}  // namespace ros_gz_bridge
