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

#include "ros_ign_bridge/convert/ros_ign_interfaces.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::JointWrench & ros_msg,
  ignition::msgs::JointWrench & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  ign_msg.set_body_1_name(ros_msg.body_1_name.data);
  ign_msg.set_body_2_name(ros_msg.body_2_name.data);
  ign_msg.set_body_1_id(ros_msg.body_1_id.data);
  ign_msg.set_body_2_id(ros_msg.body_2_id.data);
  convert_ros_to_ign(ros_msg.body_1_wrench, (*ign_msg.mutable_body_1_wrench()));
  convert_ros_to_ign(ros_msg.body_2_wrench, (*ign_msg.mutable_body_2_wrench()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::JointWrench & ign_msg,
  ros_ign_interfaces::msg::JointWrench & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  ros_msg.body_1_name.data = ign_msg.body_1_name();
  ros_msg.body_2_name.data = ign_msg.body_2_name();
  ros_msg.body_1_id.data = ign_msg.body_1_id();
  ros_msg.body_2_id.data = ign_msg.body_2_id();
  convert_ign_to_ros(ign_msg.body_1_wrench(), ros_msg.body_1_wrench);
  convert_ign_to_ros(ign_msg.body_2_wrench(), ros_msg.body_2_wrench);
}

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Entity & ros_msg,
  ignition::msgs::Entity & ign_msg)
{
  ign_msg.set_id(ros_msg.id);
  ign_msg.set_name(ros_msg.name);
  switch (ros_msg.type) {
    case ros_ign_interfaces::msg::Entity::NONE:
      ign_msg.set_type(ignition::msgs::Entity::NONE);
      break;
    case ros_ign_interfaces::msg::Entity::LIGHT:
      ign_msg.set_type(ignition::msgs::Entity::LIGHT);
      break;
    case ros_ign_interfaces::msg::Entity::MODEL:
      ign_msg.set_type(ignition::msgs::Entity::MODEL);
      break;
    case ros_ign_interfaces::msg::Entity::LINK:
      ign_msg.set_type(ignition::msgs::Entity::LINK);
      break;
    case ros_ign_interfaces::msg::Entity::VISUAL:
      ign_msg.set_type(ignition::msgs::Entity::VISUAL);
      break;
    case ros_ign_interfaces::msg::Entity::COLLISION:
      ign_msg.set_type(ignition::msgs::Entity::COLLISION);
      break;
    case ros_ign_interfaces::msg::Entity::SENSOR:
      ign_msg.set_type(ignition::msgs::Entity::SENSOR);
      break;
    case ros_ign_interfaces::msg::Entity::JOINT:
      ign_msg.set_type(ignition::msgs::Entity::JOINT);
      break;
    default:
      std::cerr << "Unsupported entity type [" << ros_msg.type << "]\n";
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Entity & ign_msg,
  ros_ign_interfaces::msg::Entity & ros_msg)
{
  ros_msg.id = ign_msg.id();
  ros_msg.name = ign_msg.name();
  if (ign_msg.type() == ignition::msgs::Entity::Type::Entity_Type_NONE) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::NONE;
  } else if (ign_msg.type() == ignition::msgs::Entity::LIGHT) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::LIGHT;
  } else if (ign_msg.type() == ignition::msgs::Entity::MODEL) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::MODEL;
  } else if (ign_msg.type() == ignition::msgs::Entity::LINK) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::LINK;
  } else if (ign_msg.type() == ignition::msgs::Entity::VISUAL) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::VISUAL;
  } else if (ign_msg.type() == ignition::msgs::Entity::COLLISION) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::COLLISION;
  } else if (ign_msg.type() == ignition::msgs::Entity::SENSOR) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::SENSOR;
  } else if (ign_msg.type() == ignition::msgs::Entity::JOINT) {
    ros_msg.type = ros_ign_interfaces::msg::Entity::JOINT;
  } else {
    std::cerr << "Unsupported Entity [" <<
      ign_msg.type() << "]" << std::endl;
  }
}

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Contact & ros_msg,
  ignition::msgs::Contact & ign_msg)
{
  convert_ros_to_ign(ros_msg.collision1, (*ign_msg.mutable_collision1()));
  convert_ros_to_ign(ros_msg.collision1, (*ign_msg.mutable_collision2()));
  ign_msg.clear_position();
  for (auto const & ros_position : ros_msg.positions) {
    auto ign_position = ign_msg.add_position();
    convert_ros_to_ign(ros_position, *ign_position);
  }
  ign_msg.clear_normal();
  for (const auto & ros_normal : ros_msg.normals) {
    auto ign_normal = ign_msg.add_normal();
    convert_ros_to_ign(ros_normal, *ign_normal);
  }
  for (const auto & ros_depth : ros_msg.depths) {
    ign_msg.add_depth(ros_depth);
  }
  ign_msg.clear_wrench();
  for (const auto & ros_wrench : ros_msg.wrenches) {
    auto ign_wrench = ign_msg.add_wrench();
    convert_ros_to_ign(ros_wrench, *ign_wrench);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Contact & ign_msg,
  ros_ign_interfaces::msg::Contact & ros_msg)
{
  convert_ign_to_ros(ign_msg.collision1(), ros_msg.collision1);
  convert_ign_to_ros(ign_msg.collision2(), ros_msg.collision2);
  for (auto i = 0; i < ign_msg.position_size(); ++i) {
    geometry_msgs::msg::Vector3 ros_position;
    convert_ign_to_ros(ign_msg.position(i), ros_position);
    ros_msg.positions.push_back(ros_position);
  }
  for (auto i = 0; i < ign_msg.normal_size(); ++i) {
    geometry_msgs::msg::Vector3 ros_normal;
    convert_ign_to_ros(ign_msg.normal(i), ros_normal);
    ros_msg.normals.push_back(ros_normal);
  }
  for (auto i = 0; i < ign_msg.depth_size(); ++i) {
    ros_msg.depths.push_back(ign_msg.depth(i));
  }
  for (auto i = 0; i < ign_msg.wrench_size(); ++i) {
    ros_ign_interfaces::msg::JointWrench ros_joint_wrench;
    convert_ign_to_ros(ign_msg.wrench(i), ros_joint_wrench);
    ros_msg.wrenches.push_back(ros_joint_wrench);
  }
}

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Contacts & ros_msg,
  ignition::msgs::Contacts & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  ign_msg.clear_contact();
  for (const auto & ros_contact : ros_msg.contacts) {
    auto ign_contact = ign_msg.add_contact();
    convert_ros_to_ign(ros_contact, *ign_contact);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Contacts & ign_msg,
  ros_ign_interfaces::msg::Contacts & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  for (auto i = 0; i < ign_msg.contact_size(); ++i) {
    ros_ign_interfaces::msg::Contact ros_contact;
    convert_ign_to_ros(ign_msg.contact(i), ros_contact);
    ros_msg.contacts.push_back(ros_contact);
  }
}

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::msg::Light & ros_msg,
  ignition::msgs::Light & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_name(ros_msg.name);
  if (ros_msg.type == 0) {
    ign_msg.set_type(ignition::msgs::Light_LightType::Light_LightType_POINT);
  } else if (ros_msg.type == 1) {
    ign_msg.set_type(ignition::msgs::Light_LightType::Light_LightType_SPOT);
  } else if (ros_msg.type == 2) {
    ign_msg.set_type(
      ignition::msgs::Light_LightType::Light_LightType_DIRECTIONAL);
  }

  convert_ros_to_ign(ros_msg.pose, *ign_msg.mutable_pose());
  convert_ros_to_ign(ros_msg.diffuse, *ign_msg.mutable_diffuse());
  convert_ros_to_ign(ros_msg.specular, *ign_msg.mutable_specular());
  ign_msg.set_attenuation_constant(ros_msg.attenuation_constant);
  ign_msg.set_attenuation_linear(ros_msg.attenuation_linear);
  ign_msg.set_attenuation_quadratic(ros_msg.attenuation_quadratic);
  convert_ros_to_ign(ros_msg.direction, *ign_msg.mutable_direction());
  ign_msg.set_range(ros_msg.range);
  ign_msg.set_cast_shadows(ros_msg.cast_shadows);
  ign_msg.set_spot_inner_angle(ros_msg.spot_inner_angle);
  ign_msg.set_spot_outer_angle(ros_msg.spot_outer_angle);
  ign_msg.set_spot_falloff(ros_msg.spot_falloff);

  ign_msg.set_id(ros_msg.id);

  ign_msg.set_parent_id(ros_msg.parent_id);

  ign_msg.set_intensity(ros_msg.intensity);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Light & ign_msg,
  ros_ign_interfaces::msg::Light & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  ros_msg.name = ign_msg.name();
  if (ign_msg.type() ==
    ignition::msgs::Light_LightType::Light_LightType_POINT)
  {
    ros_msg.type = 0;
  } else if (ign_msg.type() == ignition::msgs::Light_LightType::Light_LightType_SPOT) {
    ros_msg.type = 1;
  } else if (ign_msg.type() == ignition::msgs::Light_LightType::Light_LightType_DIRECTIONAL) {
    ros_msg.type = 2;
  }

  convert_ign_to_ros(ign_msg.pose(), ros_msg.pose);
  convert_ign_to_ros(ign_msg.diffuse(), ros_msg.diffuse);
  convert_ign_to_ros(ign_msg.specular(), ros_msg.specular);
  ros_msg.attenuation_constant = ign_msg.attenuation_constant();
  ros_msg.attenuation_linear = ign_msg.attenuation_linear();
  ros_msg.attenuation_quadratic = ign_msg.attenuation_quadratic();
  convert_ign_to_ros(ign_msg.direction(), ros_msg.direction);
  ros_msg.range = ign_msg.range();
  ros_msg.cast_shadows = ign_msg.cast_shadows();
  ros_msg.spot_inner_angle = ign_msg.spot_inner_angle();
  ros_msg.spot_outer_angle = ign_msg.spot_outer_angle();
  ros_msg.spot_falloff = ign_msg.spot_falloff();

  ros_msg.id = ign_msg.id();

  ros_msg.parent_id = ign_msg.parent_id();

  ros_msg.intensity = ign_msg.intensity();
}

}  // namespace ros_ign_bridge
