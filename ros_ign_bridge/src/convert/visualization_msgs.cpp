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

#include <algorithm>
#include <exception>
#include <ros/console.h>

#include "ros_ign_bridge/convert.hpp"
#include "convert/frame_id.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
    const visualization_msgs::Marker & ros_msg,
    ignition::msgs::Marker & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  // Note, in ROS's Marker message ADD and MODIFY both map to a value of "0", 
  // so that case is not needed here.
  switch(ros_msg.action)
  {
    case visualization_msgs::Marker::ADD:
      ign_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      break;
    case visualization_msgs::Marker::DELETE:
      ign_msg.set_action(ignition::msgs::Marker::DELETE_MARKER);
      break;
    case visualization_msgs::Marker::DELETEALL:
      ign_msg.set_action(ignition::msgs::Marker::DELETE_ALL);
      break;
    default:
      ROS_ERROR_STREAM("Unknown visualization_msgs::Marker action [" <<
          ros_msg.action << "]\n");
      break;
  }

  ign_msg.set_ns(ros_msg.ns);
  ign_msg.set_id(ros_msg.id);
  // ign_msg.set_layer();  // No "layer" concept in ROS

  // Type
  switch(ros_msg.type)
  {
#ifdef IGNITION_DOME
    case visualization_msgs::Marker::ARROW:
      ign_msg.set_type(ignition::msgs::Marker::ARROW);
      break;
#endif
    case visualization_msgs::Marker::CUBE:
      ign_msg.set_type(ignition::msgs::Marker::BOX);
      break;
    case visualization_msgs::Marker::SPHERE:
      ign_msg.set_type(ignition::msgs::Marker::SPHERE);
      break;
    case visualization_msgs::Marker::CYLINDER:
      ign_msg.set_type(ignition::msgs::Marker::CYLINDER);
      break;
    case visualization_msgs::Marker::LINE_STRIP:
      ign_msg.set_type(ignition::msgs::Marker::LINE_STRIP);
      break;
    case visualization_msgs::Marker::LINE_LIST:
      ign_msg.set_type(ignition::msgs::Marker::LINE_LIST);
      break;
    case visualization_msgs::Marker::CUBE_LIST:
      ROS_ERROR_STREAM("Unsupported visualization_msgs::Marker type" <<
          "[CUBE_LIST]\n");
      break;
    case visualization_msgs::Marker::SPHERE_LIST:
      ROS_ERROR_STREAM("Unsupported visualization_msgs::Marker type" <<
          "[SPHERE_LIST]\n");
      break;
    case visualization_msgs::Marker::POINTS:
      ign_msg.set_type(ignition::msgs::Marker::POINTS);
      break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
      ign_msg.set_type(ignition::msgs::Marker::TEXT);
      break;
    case visualization_msgs::Marker::MESH_RESOURCE:
      ROS_ERROR_STREAM("Unsupported visualization_msgs::Marker type" <<
          "[MESH_RESOURCE]\n");
      break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
      ign_msg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
      break;
    default:
      ROS_ERROR_STREAM("Unknown visualization_msgs::Marker type [" <<
          ros_msg.type << "]\n");
      break;
  }

  // Lifetime
  ign_msg.mutable_lifetime()->set_sec(ros_msg.lifetime.sec);
  ign_msg.mutable_lifetime()->set_nsec(ros_msg.lifetime.nsec);

  // Pose
  convert_ros_to_ign(ros_msg.pose, *ign_msg.mutable_pose());

  // Scale
  convert_ros_to_ign(ros_msg.scale, *ign_msg.mutable_scale());

  // Material
  convert_ros_to_ign(ros_msg.color, *ign_msg.mutable_material()->mutable_ambient());
  convert_ros_to_ign(ros_msg.color, *ign_msg.mutable_material()->mutable_diffuse());
  convert_ros_to_ign(ros_msg.color, *ign_msg.mutable_material()->mutable_specular());

  // Point
  ign_msg.clear_point();
  for (auto const &pt : ros_msg.points)
  {
    auto p = ign_msg.add_point();
    convert_ros_to_ign(pt, *p);
  }

  ign_msg.set_text(ros_msg.text);

  // ign_msg.set_parent();  // No "parent" concept in ROS
  // ign_msg.set_visibility();  // No "visibility" concept in ROS
}

template<>
void
convert_ign_to_ros(
    const ignition::msgs::Marker & ign_msg,
    visualization_msgs::Marker & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  switch(ign_msg.action())
  {
    case ignition::msgs::Marker::ADD_MODIFY:
      ros_msg.action = visualization_msgs::Marker::ADD;
      break;
    case ignition::msgs::Marker::DELETE_MARKER:
      ros_msg.action = visualization_msgs::Marker::DELETE;
      break;
    case ignition::msgs::Marker::DELETE_ALL:
      ros_msg.action = visualization_msgs::Marker::DELETEALL;
      break;
    default:
      ROS_ERROR_STREAM("Unknown ignition.msgs.marker action [" <<
          ign_msg.action() << "]\n");
      break;
  }

  ros_msg.ns = ign_msg.ns();
  ros_msg.id = ign_msg.id();

  switch(ign_msg.type())
  {
#ifdef IGNITION_DOME
    case ignition::msgs::Marker::ARROW:
      ros_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
      break;
    case ignition::msgs::Marker::AXIS:
      ROS_ERROR_STREAM("Unsupported ignition.msgs.marker type " <<
          "[AXIS]\n");
      break;
    case ignition::msgs::Marker::CONE:
      ROS_ERROR_STREAM("Unsupported ignition.msgs.marker type " <<
          "[CONE]\n");
      break;
#endif
    case ignition::msgs::Marker::NONE:
      ROS_ERROR_STREAM("Unsupported ignition.msgs.marker type " <<
          "[NONE]\n");
      break;
    case ignition::msgs::Marker::BOX:
      ros_msg.type = visualization_msgs::Marker::CUBE;
      break;
    case ignition::msgs::Marker::CYLINDER:
      ros_msg.type = visualization_msgs::Marker::CYLINDER;
      break;
    case ignition::msgs::Marker::LINE_LIST:
      ros_msg.type = visualization_msgs::Marker::LINE_LIST;
      break;
    case ignition::msgs::Marker::LINE_STRIP:
      ros_msg.type = visualization_msgs::Marker::LINE_STRIP;
      break;
    case ignition::msgs::Marker::POINTS:
      ros_msg.type = visualization_msgs::Marker::POINTS;
      break;
    case ignition::msgs::Marker::SPHERE:
      ros_msg.type = visualization_msgs::Marker::SPHERE;
      break;
    case ignition::msgs::Marker::TEXT:
      ros_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      break;
    case ignition::msgs::Marker::TRIANGLE_FAN:
      ROS_ERROR_STREAM("Unsupported ignition.msgs.marker type " <<
          "[TRIANGLE_FAN]\n");
      break;
    case ignition::msgs::Marker::TRIANGLE_LIST:
      ros_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
      break;
    case ignition::msgs::Marker::TRIANGLE_STRIP:
      ROS_ERROR_STREAM("Unsupported ignition.msgs.marker type " <<
          "[TRIANGLE_STRIP]\n");
      break;
    default:
      ROS_ERROR_STREAM("Unknown ignition.msgs.marker type " <<
          "[" << ign_msg.type() << "]\n");
      break;
  }

  // Lifetime
  ros_msg.lifetime.sec = ign_msg.lifetime().sec();
  ros_msg.lifetime.nsec = ign_msg.lifetime().nsec();

  // Pose
  convert_ign_to_ros(ign_msg.pose(), ros_msg.pose);

  // Scale
  convert_ign_to_ros(ign_msg.scale(), ros_msg.scale);

  // Material
  convert_ign_to_ros(ign_msg.material().ambient(), ros_msg.color);
  ros_msg.colors.clear();

  // Points
  ros_msg.points.clear();
  ros_msg.points.reserve(ign_msg.point_size());
  for (auto const & pt: ign_msg.point())
  {
    geometry_msgs::Point p;
    convert_ign_to_ros(pt, p);
    ros_msg.points.push_back(p);
  }

  ros_msg.text = ign_msg.text();
}

template<>
void
convert_ros_to_ign(
    const visualization_msgs::MarkerArray & ros_msg,
    ignition::msgs::Marker_V & ign_msg)
{
  ign_msg.clear_header();
  ign_msg.clear_marker();
  for (const auto &marker : ros_msg.markers)
  {
    auto m = ign_msg.add_marker();
    convert_ros_to_ign(marker, *m);
  }
}

template<>
void
convert_ign_to_ros(
    const ignition::msgs::Marker_V & ign_msg,
    visualization_msgs::MarkerArray & ros_msg)
{
  ros_msg.markers.clear();
  ros_msg.markers.reserve(ign_msg.marker_size());

  for (auto const &marker : ign_msg.marker())
  {
      visualization_msgs::Marker m;
      convert_ign_to_ros(marker, m);
      ros_msg.markers.push_back(m);
  }
}

}  // namespace ros_ign_bridge

