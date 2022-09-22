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

#ifndef ROS_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_
#define ROS_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_

// Gazebo Msgs
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/joint_wrench.pb.h>
#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/gui_camera.pb.h>
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/param_v.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/msgs/track_visual.pb.h>
#include <ignition/msgs/video_record.pb.h>
#include <ignition/msgs/world_control.pb.h>

// ROS 2 messages
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/msg/joint_wrench.hpp>
#include <ros_gz_interfaces/msg/contact.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <ros_gz_interfaces/msg/gui_camera.hpp>
#include <ros_gz_interfaces/msg/light.hpp>
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <ros_gz_interfaces/msg/string_vec.hpp>
#include <ros_gz_interfaces/msg/track_visual.hpp>
#include <ros_gz_interfaces/msg/video_record.hpp>
#include <ros_gz_interfaces/msg/world_control.hpp>

// Required for HAVE_DATAFRAME definition
#include <ros_gz_bridge/ros_gz_bridge.hpp>

#if HAVE_DATAFRAME
#include <ignition/msgs/dataframe.pb.h>
#include <ros_gz_interfaces/msg/dataframe.hpp>
#endif  // HAVE_DATAFRAME

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::JointWrench & ros_msg,
  ignition::msgs::JointWrench & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::JointWrench & gz_msg,
  ros_gz_interfaces::msg::JointWrench & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  ignition::msgs::Entity & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contact & ros_msg,
  ignition::msgs::Contact & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Contact & gz_msg,
  ros_gz_interfaces::msg::Contact & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contacts & ros_msg,
  ignition::msgs::Contacts & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Contacts & gz_msg,
  ros_gz_interfaces::msg::Contacts & ros_msg);

#if HAVE_DATAFRAME
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Dataframe & ros_msg,
  ignition::msgs::Dataframe & ign_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Dataframe & ign_msg,
  ros_gz_interfaces::msg::Dataframe & ros_msg);
#endif  // HAVE_DATAFRAME

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::GuiCamera & ros_msg,
  ignition::msgs::GUICamera & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::GUICamera & gz_msg,
  ros_gz_interfaces::msg::GuiCamera & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Light & ros_msg,
  ignition::msgs::Light & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Light & gz_msg,
  ros_gz_interfaces::msg::Light & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::StringVec & ros_msg,
  ignition::msgs::StringMsg_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::StringMsg_V & gz_msg,
  ros_gz_interfaces::msg::StringVec & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  ignition::msgs::Param & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Param & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  ignition::msgs::Param_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Param_V & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::TrackVisual & ros_msg,
  ignition::msgs::TrackVisual & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::TrackVisual & gz_msg,
  ros_gz_interfaces::msg::TrackVisual & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::VideoRecord & ros_msg,
  ignition::msgs::VideoRecord & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::VideoRecord & gz_msg,
  ros_gz_interfaces::msg::VideoRecord & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::WorldControl & ros_msg,
  ignition::msgs::WorldControl & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::WorldControl & gz_msg,
  ros_gz_interfaces::msg::WorldControl & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::WorldReset & ros_msg,
  ignition::msgs::WorldReset & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::WorldReset & gz_msg,
  ros_gz_interfaces::msg::WorldReset & ros_msg);
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_
