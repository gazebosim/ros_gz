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

#include "ros_gz_bridge/convert/ros_gz_interfaces.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::JointWrench & ros_msg,
  ignition::msgs::JointWrench & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_body_1_name(ros_msg.body_1_name.data);
  gz_msg.set_body_2_name(ros_msg.body_2_name.data);
  gz_msg.set_body_1_id(ros_msg.body_1_id.data);
  gz_msg.set_body_2_id(ros_msg.body_2_id.data);
  convert_ros_to_ign(ros_msg.body_1_wrench, (*gz_msg.mutable_body_1_wrench()));
  convert_ros_to_ign(ros_msg.body_2_wrench, (*gz_msg.mutable_body_2_wrench()));
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::JointWrench & gz_msg,
  ros_gz_interfaces::msg::JointWrench & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.body_1_name.data = gz_msg.body_1_name();
  ros_msg.body_2_name.data = gz_msg.body_2_name();
  ros_msg.body_1_id.data = gz_msg.body_1_id();
  ros_msg.body_2_id.data = gz_msg.body_2_id();
  convert_gz_to_ros(gz_msg.body_1_wrench(), ros_msg.body_1_wrench);
  convert_gz_to_ros(gz_msg.body_2_wrench(), ros_msg.body_2_wrench);
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  ignition::msgs::Entity & gz_msg)
{
  gz_msg.set_id(ros_msg.id);
  gz_msg.set_name(ros_msg.name);
  switch (ros_msg.type) {
    case ros_gz_interfaces::msg::Entity::NONE:
      gz_msg.set_type(ignition::msgs::Entity::NONE);
      break;
    case ros_gz_interfaces::msg::Entity::LIGHT:
      gz_msg.set_type(ignition::msgs::Entity::LIGHT);
      break;
    case ros_gz_interfaces::msg::Entity::MODEL:
      gz_msg.set_type(ignition::msgs::Entity::MODEL);
      break;
    case ros_gz_interfaces::msg::Entity::LINK:
      gz_msg.set_type(ignition::msgs::Entity::LINK);
      break;
    case ros_gz_interfaces::msg::Entity::VISUAL:
      gz_msg.set_type(ignition::msgs::Entity::VISUAL);
      break;
    case ros_gz_interfaces::msg::Entity::COLLISION:
      gz_msg.set_type(ignition::msgs::Entity::COLLISION);
      break;
    case ros_gz_interfaces::msg::Entity::SENSOR:
      gz_msg.set_type(ignition::msgs::Entity::SENSOR);
      break;
    case ros_gz_interfaces::msg::Entity::JOINT:
      gz_msg.set_type(ignition::msgs::Entity::JOINT);
      break;
    default:
      std::cerr << "Unsupported entity type [" << ros_msg.type << "]\n";
  }
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg)
{
  ros_msg.id = gz_msg.id();
  ros_msg.name = gz_msg.name();
  if (gz_msg.type() == ignition::msgs::Entity::Type::Entity_Type_NONE) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::NONE;
  } else if (gz_msg.type() == ignition::msgs::Entity::LIGHT) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::LIGHT;
  } else if (gz_msg.type() == ignition::msgs::Entity::MODEL) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::MODEL;
  } else if (gz_msg.type() == ignition::msgs::Entity::LINK) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::LINK;
  } else if (gz_msg.type() == ignition::msgs::Entity::VISUAL) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::VISUAL;
  } else if (gz_msg.type() == ignition::msgs::Entity::COLLISION) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::COLLISION;
  } else if (gz_msg.type() == ignition::msgs::Entity::SENSOR) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::SENSOR;
  } else if (gz_msg.type() == ignition::msgs::Entity::JOINT) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::JOINT;
  } else {
    std::cerr << "Unsupported Entity [" <<
      gz_msg.type() << "]" << std::endl;
  }
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::Contact & ros_msg,
  ignition::msgs::Contact & gz_msg)
{
  convert_ros_to_ign(ros_msg.collision1, (*gz_msg.mutable_collision1()));
  convert_ros_to_ign(ros_msg.collision1, (*gz_msg.mutable_collision2()));
  gz_msg.clear_position();
  for (auto const & ros_position : ros_msg.positions) {
    auto gz_position = gz_msg.add_position();
    convert_ros_to_ign(ros_position, *gz_position);
  }
  gz_msg.clear_normal();
  for (const auto & ros_normal : ros_msg.normals) {
    auto gz_normal = gz_msg.add_normal();
    convert_ros_to_ign(ros_normal, *gz_normal);
  }
  for (const auto & ros_depth : ros_msg.depths) {
    gz_msg.add_depth(ros_depth);
  }
  gz_msg.clear_wrench();
  for (const auto & ros_wrench : ros_msg.wrenches) {
    auto gz_wrench = gz_msg.add_wrench();
    convert_ros_to_ign(ros_wrench, *gz_wrench);
  }
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Contact & gz_msg,
  ros_gz_interfaces::msg::Contact & ros_msg)
{
  convert_gz_to_ros(gz_msg.collision1(), ros_msg.collision1);
  convert_gz_to_ros(gz_msg.collision2(), ros_msg.collision2);
  for (auto i = 0; i < gz_msg.position_size(); ++i) {
    geometry_msgs::msg::Vector3 ros_position;
    convert_gz_to_ros(gz_msg.position(i), ros_position);
    ros_msg.positions.push_back(ros_position);
  }
  for (auto i = 0; i < gz_msg.normal_size(); ++i) {
    geometry_msgs::msg::Vector3 ros_normal;
    convert_gz_to_ros(gz_msg.normal(i), ros_normal);
    ros_msg.normals.push_back(ros_normal);
  }
  for (auto i = 0; i < gz_msg.depth_size(); ++i) {
    ros_msg.depths.push_back(gz_msg.depth(i));
  }
  for (auto i = 0; i < gz_msg.wrench_size(); ++i) {
    ros_gz_interfaces::msg::JointWrench ros_joint_wrench;
    convert_gz_to_ros(gz_msg.wrench(i), ros_joint_wrench);
    ros_msg.wrenches.push_back(ros_joint_wrench);
  }
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::Contacts & ros_msg,
  ignition::msgs::Contacts & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.clear_contact();
  for (const auto & ros_contact : ros_msg.contacts) {
    auto gz_contact = gz_msg.add_contact();
    convert_ros_to_ign(ros_contact, *gz_contact);
  }
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Contacts & gz_msg,
  ros_gz_interfaces::msg::Contacts & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  for (auto i = 0; i < gz_msg.contact_size(); ++i) {
    ros_gz_interfaces::msg::Contact ros_contact;
    convert_gz_to_ros(gz_msg.contact(i), ros_contact);
    ros_msg.contacts.push_back(ros_contact);
  }
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::GuiCamera & ros_msg,
  ignition::msgs::GUICamera & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, *gz_msg.mutable_header());
  gz_msg.set_name(ros_msg.name);
  gz_msg.set_view_controller(ros_msg.view_controller);
  convert_ros_to_ign(ros_msg.pose, *gz_msg.mutable_pose());
  convert_ros_to_ign(ros_msg.track, *gz_msg.mutable_track());
  gz_msg.set_projection_type(ros_msg.projection_type);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::GUICamera & gz_msg,
  ros_gz_interfaces::msg::GuiCamera & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.name = gz_msg.name();
  ros_msg.view_controller = gz_msg.view_controller();
  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose);
  convert_gz_to_ros(gz_msg.track(), ros_msg.track);
  ros_msg.projection_type = gz_msg.projection_type();
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::Light & ros_msg,
  ignition::msgs::Light & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_name(ros_msg.name);
  if (ros_msg.type == 0) {
    gz_msg.set_type(ignition::msgs::Light_LightType::Light_LightType_POINT);
  } else if (ros_msg.type == 1) {
    gz_msg.set_type(ignition::msgs::Light_LightType::Light_LightType_SPOT);
  } else if (ros_msg.type == 2) {
    gz_msg.set_type(
      ignition::msgs::Light_LightType::Light_LightType_DIRECTIONAL);
  }

  convert_ros_to_ign(ros_msg.pose, *gz_msg.mutable_pose());
  convert_ros_to_ign(ros_msg.diffuse, *gz_msg.mutable_diffuse());
  convert_ros_to_ign(ros_msg.specular, *gz_msg.mutable_specular());
  gz_msg.set_attenuation_constant(ros_msg.attenuation_constant);
  gz_msg.set_attenuation_linear(ros_msg.attenuation_linear);
  gz_msg.set_attenuation_quadratic(ros_msg.attenuation_quadratic);
  convert_ros_to_ign(ros_msg.direction, *gz_msg.mutable_direction());
  gz_msg.set_range(ros_msg.range);
  gz_msg.set_cast_shadows(ros_msg.cast_shadows);
  gz_msg.set_spot_inner_angle(ros_msg.spot_inner_angle);
  gz_msg.set_spot_outer_angle(ros_msg.spot_outer_angle);
  gz_msg.set_spot_falloff(ros_msg.spot_falloff);

  gz_msg.set_id(ros_msg.id);

  gz_msg.set_parent_id(ros_msg.parent_id);

  gz_msg.set_intensity(ros_msg.intensity);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Light & gz_msg,
  ros_gz_interfaces::msg::Light & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.name = gz_msg.name();
  if (gz_msg.type() ==
    ignition::msgs::Light_LightType::Light_LightType_POINT)
  {
    ros_msg.type = 0;
  } else if (gz_msg.type() == ignition::msgs::Light_LightType::Light_LightType_SPOT) {
    ros_msg.type = 1;
  } else if (gz_msg.type() == ignition::msgs::Light_LightType::Light_LightType_DIRECTIONAL) {
    ros_msg.type = 2;
  }

  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose);
  convert_gz_to_ros(gz_msg.diffuse(), ros_msg.diffuse);
  convert_gz_to_ros(gz_msg.specular(), ros_msg.specular);
  ros_msg.attenuation_constant = gz_msg.attenuation_constant();
  ros_msg.attenuation_linear = gz_msg.attenuation_linear();
  ros_msg.attenuation_quadratic = gz_msg.attenuation_quadratic();
  convert_gz_to_ros(gz_msg.direction(), ros_msg.direction);
  ros_msg.range = gz_msg.range();
  ros_msg.cast_shadows = gz_msg.cast_shadows();
  ros_msg.spot_inner_angle = gz_msg.spot_inner_angle();
  ros_msg.spot_outer_angle = gz_msg.spot_outer_angle();
  ros_msg.spot_falloff = gz_msg.spot_falloff();

  ros_msg.id = gz_msg.id();

  ros_msg.parent_id = gz_msg.parent_id();

  ros_msg.intensity = gz_msg.intensity();
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::StringVec & ros_msg,
  ignition::msgs::StringMsg_V & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, *gz_msg.mutable_header());
  for (const auto & elem : ros_msg.data) {
    auto * new_elem = gz_msg.add_data();
    *new_elem = elem;
  }
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::StringMsg_V & gz_msg,
  ros_gz_interfaces::msg::StringVec & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  for (const auto & elem : gz_msg.data()) {
    ros_msg.data.emplace_back(elem);
  }
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::TrackVisual & ros_msg,
  ignition::msgs::TrackVisual & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, *gz_msg.mutable_header());
  gz_msg.set_name(ros_msg.name);
  gz_msg.set_id(ros_msg.id);
  gz_msg.set_inherit_orientation(ros_msg.inherit_orientation);
  gz_msg.set_min_dist(ros_msg.min_dist);
  gz_msg.set_max_dist(ros_msg.max_dist);
  gz_msg.set_static_(ros_msg.is_static);
  gz_msg.set_use_model_frame(ros_msg.use_model_frame);
  convert_ros_to_ign(ros_msg.xyz, *gz_msg.mutable_xyz());
  gz_msg.set_inherit_yaw(ros_msg.inherit_yaw);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::TrackVisual & gz_msg,
  ros_gz_interfaces::msg::TrackVisual & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.name = gz_msg.name();
  ros_msg.id = gz_msg.id();
  ros_msg.inherit_orientation = gz_msg.inherit_orientation();
  ros_msg.min_dist = gz_msg.min_dist();
  ros_msg.max_dist = gz_msg.max_dist();
  ros_msg.is_static = gz_msg.static_();
  ros_msg.use_model_frame = gz_msg.use_model_frame();
  convert_gz_to_ros(gz_msg.xyz(), ros_msg.xyz);
  ros_msg.inherit_yaw = gz_msg.inherit_yaw();
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::VideoRecord & ros_msg,
  ignition::msgs::VideoRecord & gz_msg)
{
  convert_ros_to_ign(ros_msg.header, *gz_msg.mutable_header());
  gz_msg.set_start(ros_msg.start);
  gz_msg.set_stop(ros_msg.stop);
  gz_msg.set_format(ros_msg.format);
  gz_msg.set_save_filename(ros_msg.save_filename);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::VideoRecord & gz_msg,
  ros_gz_interfaces::msg::VideoRecord & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.start = gz_msg.start();
  ros_msg.stop = gz_msg.stop();
  ros_msg.format = gz_msg.format();
  ros_msg.save_filename = gz_msg.save_filename();
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::WorldControl & ros_msg,
  ignition::msgs::WorldControl & gz_msg)
{
  gz_msg.set_pause(ros_msg.pause);
  gz_msg.set_step(ros_msg.step);
  gz_msg.set_multi_step(ros_msg.multi_step);
  convert_ros_to_ign(ros_msg.reset, *gz_msg.mutable_reset());
  gz_msg.set_seed(ros_msg.seed);
  convert_ros_to_ign(ros_msg.run_to_sim_time, *gz_msg.mutable_run_to_sim_time());
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::WorldControl & gz_msg,
  ros_gz_interfaces::msg::WorldControl & ros_msg)
{
  ros_msg.pause = gz_msg.pause();
  ros_msg.step = gz_msg.step();
  ros_msg.multi_step = gz_msg.multi_step();
  convert_gz_to_ros(gz_msg.reset(), ros_msg.reset);
  ros_msg.seed = gz_msg.seed();
  convert_gz_to_ros(gz_msg.run_to_sim_time(), ros_msg.run_to_sim_time);
}

template<>
void
convert_ros_to_ign(
  const ros_gz_interfaces::msg::WorldReset & ros_msg,
  ignition::msgs::WorldReset & gz_msg)
{
  gz_msg.set_all(ros_msg.all);
  gz_msg.set_time_only(ros_msg.time_only);
  gz_msg.set_model_only(ros_msg.model_only);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::WorldReset & gz_msg,
  ros_gz_interfaces::msg::WorldReset & ros_msg)
{
  ros_msg.all = gz_msg.all();
  ros_msg.time_only = gz_msg.time_only();
  ros_msg.model_only = gz_msg.model_only();
}
}  // namespace ros_gz_bridge
