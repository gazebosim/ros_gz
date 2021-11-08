// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:Color.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__DETAIL__COLOR__TRAITS_HPP_
#define ROS_IGN_INTERFACES__DETAIL__COLOR__TRAITS_HPP_

#include "ros_ign_interfaces/msg/detail/color__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const ros_ign_interfaces::msg::Color & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g: ";
    value_to_yaml(msg.g, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    value_to_yaml(msg.a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ros_ign_interfaces::msg::Color & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<ros_ign_interfaces::msg::Color>()
{
  return "ros_ign_interfaces::msg::Color";
}

template<>
inline const char * name<ros_ign_interfaces::msg::Color>()
{
  return "ros_ign_interfaces/msg/Color";
}

template<>
struct has_fixed_size<ros_ign_interfaces::msg::Color>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_ign_interfaces::msg::Color>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_ign_interfaces::msg::Color>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__DETAIL__COLOR__TRAITS_HPP_
