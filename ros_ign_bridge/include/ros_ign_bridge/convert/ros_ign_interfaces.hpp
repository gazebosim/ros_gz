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

#ifndef ROS_IGN_BRIDGE__CONVERT__ROS_IGN_INTERFACES_HPP_
#define ROS_IGN_BRIDGE__CONVERT__ROS_IGN_INTERFACES_HPP_

// Required for HAVE_DATAFRAME definition
#include <ros_ign_bridge/ros_ign_bridge.hpp>

// Ignition messages
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/joint_wrench.pb.h>
#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/gui_camera.pb.h>
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/msgs/track_visual.pb.h>
#include <ignition/msgs/video_record.pb.h>

// ROS 2 messages
#include <ros_ign_interfaces/msg/entity.hpp>
#include <ros_ign_interfaces/msg/joint_wrench.hpp>
#include <ros_ign_interfaces/msg/contact.hpp>
#include <ros_ign_interfaces/msg/contacts.hpp>
#include <ros_ign_interfaces/msg/gui_camera.hpp>
#include <ros_ign_interfaces/msg/light.hpp>
#include <ros_ign_interfaces/msg/param_vec.hpp>
#include <ros_ign_interfaces/msg/string_vec.hpp>
#include <ros_ign_interfaces/msg/track_visual.hpp>
#include <ros_ign_interfaces/msg/video_record.hpp>

#if HAVE_DATAFRAME
#include <ignition/msgs/dataframe.pb.h>
#include <ros_ign_interfaces/msg/dataframe.hpp>
#endif  // HAVE_DATAFRAME

#include <ros_ign_bridge/convert_decl.hpp>

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::JointWrench & ros_msg,
  ignition::msgs::JointWrench & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::JointWrench & ign_msg,
  ros_ign_interfaces::msg::JointWrench & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Entity & ros_msg,
  ignition::msgs::Entity & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Entity & ign_msg,
  ros_ign_interfaces::msg::Entity & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Contact & ros_msg,
  ignition::msgs::Contact & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Contact & ign_msg,
  ros_ign_interfaces::msg::Contact & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Contacts & ros_msg,
  ignition::msgs::Contacts & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Contacts & ign_msg,
  ros_ign_interfaces::msg::Contacts & ros_msg);

#if HAVE_DATAFRAME
template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Dataframe & ros_msg,
  ignition::msgs::Dataframe & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Dataframe & ign_msg,
  ros_ign_interfaces::msg::Dataframe & ros_msg);
#endif  // HAVE_DATAFRAME

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::GuiCamera & ros_msg,
  ignition::msgs::GUICamera & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::GUICamera & ign_msg,
  ros_ign_interfaces::msg::GuiCamera & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Light & ros_msg,
  ignition::msgs::Light & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Light & ign_msg,
  ros_ign_interfaces::msg::Light & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::StringVec & ros_msg,
  ignition::msgs::StringMsg_V & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::StringMsg_V & ign_msg,
  ros_ign_interfaces::msg::StringVec & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::ParamVec & ros_msg,
  ignition::msgs::Param & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Param & ign_msg,
  ros_ign_interfaces::msg::ParamVec & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::ParamVec & ros_msg,
  ignition::msgs::Param_V & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Param_V & ign_msg,
  ros_ign_interfaces::msg::ParamVec & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::TrackVisual & ros_msg,
  ignition::msgs::TrackVisual & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::TrackVisual & ign_msg,
  ros_ign_interfaces::msg::TrackVisual & ros_msg);

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::VideoRecord & ros_msg,
  ignition::msgs::VideoRecord & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::VideoRecord & ign_msg,
  ros_ign_interfaces::msg::VideoRecord & ros_msg);
}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT__ROS_IGN_INTERFACES_HPP_
