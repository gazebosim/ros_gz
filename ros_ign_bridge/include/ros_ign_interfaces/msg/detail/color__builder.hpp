// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:Color.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__DETAIL__COLOR__BUILDER_HPP_
#define ROS_IGN_INTERFACES__DETAIL__COLOR__BUILDER_HPP_

#include "ros_ign_interfaces/msg/detail/color__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ros_ign_interfaces
{

namespace msg
{

namespace builder
{

class Init_Color_a
{
public:
  explicit Init_Color_a(::ros_ign_interfaces::msg::Color & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::msg::Color a(::ros_ign_interfaces::msg::Color::_a_type arg)
  {
    msg_.a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Color msg_;
};

class Init_Color_b
{
public:
  explicit Init_Color_b(::ros_ign_interfaces::msg::Color & msg)
  : msg_(msg)
  {}
  Init_Color_a b(::ros_ign_interfaces::msg::Color::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_Color_a(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Color msg_;
};

class Init_Color_g
{
public:
  explicit Init_Color_g(::ros_ign_interfaces::msg::Color & msg)
  : msg_(msg)
  {}
  Init_Color_b g(::ros_ign_interfaces::msg::Color::_g_type arg)
  {
    msg_.g = std::move(arg);
    return Init_Color_b(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Color msg_;
};

class Init_Color_r
{
public:
  Init_Color_r()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Color_g r(::ros_ign_interfaces::msg::Color::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_Color_g(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Color msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::msg::Color>()
{
  return ros_ign_interfaces::msg::builder::Init_Color_r();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__DETAIL__COLOR__BUILDER_HPP_
