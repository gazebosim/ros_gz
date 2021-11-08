// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:Light.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__DETAIL__LIGHT__TRAITS_HPP_
#define ROS_IGN_INTERFACES__DETAIL__LIGHT__TRAITS_HPP_

#include "ros_ign_interfaces/msg/detail/light__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'diffuse'
// Member 'specular'
#include "ros_ign_interfaces/msg/detail/color__traits.hpp"
// Member 'direction'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const ros_ign_interfaces::msg::Light & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_yaml(msg.pose, out, indentation + 2);
  }

  // member: diffuse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diffuse:\n";
    to_yaml(msg.diffuse, out, indentation + 2);
  }

  // member: specular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "specular:\n";
    to_yaml(msg.specular, out, indentation + 2);
  }

  // member: attenuation_constant
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attenuation_constant: ";
    value_to_yaml(msg.attenuation_constant, out);
    out << "\n";
  }

  // member: attenuation_linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attenuation_linear: ";
    value_to_yaml(msg.attenuation_linear, out);
    out << "\n";
  }

  // member: attenuation_quadratic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attenuation_quadratic: ";
    value_to_yaml(msg.attenuation_quadratic, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction:\n";
    to_yaml(msg.direction, out, indentation + 2);
  }

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: cast_shadows
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cast_shadows: ";
    value_to_yaml(msg.cast_shadows, out);
    out << "\n";
  }

  // member: spot_inner_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spot_inner_angle: ";
    value_to_yaml(msg.spot_inner_angle, out);
    out << "\n";
  }

  // member: spot_outer_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spot_outer_angle: ";
    value_to_yaml(msg.spot_outer_angle, out);
    out << "\n";
  }

  // member: spot_falloff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spot_falloff: ";
    value_to_yaml(msg.spot_falloff, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: parent_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_id: ";
    value_to_yaml(msg.parent_id, out);
    out << "\n";
  }

  // member: intensity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "intensity: ";
    value_to_yaml(msg.intensity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ros_ign_interfaces::msg::Light & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<ros_ign_interfaces::msg::Light>()
{
  return "ros_ign_interfaces::msg::Light";
}

template<>
inline const char * name<ros_ign_interfaces::msg::Light>()
{
  return "ros_ign_interfaces/msg/Light";
}

template<>
struct has_fixed_size<ros_ign_interfaces::msg::Light>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_ign_interfaces::msg::Light>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_ign_interfaces::msg::Light>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__DETAIL__LIGHT__TRAITS_HPP_
