// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_ign_interfaces:Light.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__DETAIL__LIGHT__STRUCT_HPP_
#define ROS_IGN_INTERFACES__DETAIL__LIGHT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'diffuse'
// Member 'specular'
#include "ros_ign_interfaces/msg/detail/color__struct.hpp"
// Member 'direction'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_ign_interfaces__Light __attribute__((deprecated))
#else
# define DEPRECATED__ros_ign_interfaces__Light __declspec(deprecated)
#endif

namespace ros_ign_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Light_
{
  using Type = Light_<ContainerAllocator>;

  explicit Light_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    diffuse(_init),
    specular(_init),
    direction(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->type = 0;
      this->attenuation_constant = 0.0f;
      this->attenuation_linear = 0.0f;
      this->attenuation_quadratic = 0.0f;
      this->range = 0.0f;
      this->cast_shadows = false;
      this->spot_inner_angle = 0.0f;
      this->spot_outer_angle = 0.0f;
      this->spot_falloff = 0.0f;
      this->id = 0ul;
      this->parent_id = 0ul;
      this->intensity = 0.0f;
    }
  }

  explicit Light_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    pose(_alloc, _init),
    diffuse(_alloc, _init),
    specular(_alloc, _init),
    direction(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->type = 0;
      this->attenuation_constant = 0.0f;
      this->attenuation_linear = 0.0f;
      this->attenuation_quadratic = 0.0f;
      this->range = 0.0f;
      this->cast_shadows = false;
      this->spot_inner_angle = 0.0f;
      this->spot_outer_angle = 0.0f;
      this->spot_falloff = 0.0f;
      this->id = 0ul;
      this->parent_id = 0ul;
      this->intensity = 0.0f;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _type_type =
    uint8_t;
  _type_type type;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _diffuse_type =
    ros_ign_interfaces::msg::Color_<ContainerAllocator>;
  _diffuse_type diffuse;
  using _specular_type =
    ros_ign_interfaces::msg::Color_<ContainerAllocator>;
  _specular_type specular;
  using _attenuation_constant_type =
    float;
  _attenuation_constant_type attenuation_constant;
  using _attenuation_linear_type =
    float;
  _attenuation_linear_type attenuation_linear;
  using _attenuation_quadratic_type =
    float;
  _attenuation_quadratic_type attenuation_quadratic;
  using _direction_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _direction_type direction;
  using _range_type =
    float;
  _range_type range;
  using _cast_shadows_type =
    bool;
  _cast_shadows_type cast_shadows;
  using _spot_inner_angle_type =
    float;
  _spot_inner_angle_type spot_inner_angle;
  using _spot_outer_angle_type =
    float;
  _spot_outer_angle_type spot_outer_angle;
  using _spot_falloff_type =
    float;
  _spot_falloff_type spot_falloff;
  using _id_type =
    uint32_t;
  _id_type id;
  using _parent_id_type =
    uint32_t;
  _parent_id_type parent_id;
  using _intensity_type =
    float;
  _intensity_type intensity;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__diffuse(
    const ros_ign_interfaces::msg::Color_<ContainerAllocator> & _arg)
  {
    this->diffuse = _arg;
    return *this;
  }
  Type & set__specular(
    const ros_ign_interfaces::msg::Color_<ContainerAllocator> & _arg)
  {
    this->specular = _arg;
    return *this;
  }
  Type & set__attenuation_constant(
    const float & _arg)
  {
    this->attenuation_constant = _arg;
    return *this;
  }
  Type & set__attenuation_linear(
    const float & _arg)
  {
    this->attenuation_linear = _arg;
    return *this;
  }
  Type & set__attenuation_quadratic(
    const float & _arg)
  {
    this->attenuation_quadratic = _arg;
    return *this;
  }
  Type & set__direction(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__range(
    const float & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__cast_shadows(
    const bool & _arg)
  {
    this->cast_shadows = _arg;
    return *this;
  }
  Type & set__spot_inner_angle(
    const float & _arg)
  {
    this->spot_inner_angle = _arg;
    return *this;
  }
  Type & set__spot_outer_angle(
    const float & _arg)
  {
    this->spot_outer_angle = _arg;
    return *this;
  }
  Type & set__spot_falloff(
    const float & _arg)
  {
    this->spot_falloff = _arg;
    return *this;
  }
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__parent_id(
    const uint32_t & _arg)
  {
    this->parent_id = _arg;
    return *this;
  }
  Type & set__intensity(
    const float & _arg)
  {
    this->intensity = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t POINT =
    0u;
  static constexpr uint8_t SPOT =
    1u;
  static constexpr uint8_t DIRECTIONAL =
    2u;

  // pointer types
  using RawPtr =
    ros_ign_interfaces::msg::Light_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_ign_interfaces::msg::Light_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_ign_interfaces::msg::Light_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_ign_interfaces::msg::Light_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_ign_interfaces__Light
    std::shared_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_ign_interfaces__Light
    std::shared_ptr<ros_ign_interfaces::msg::Light_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Light_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->diffuse != other.diffuse) {
      return false;
    }
    if (this->specular != other.specular) {
      return false;
    }
    if (this->attenuation_constant != other.attenuation_constant) {
      return false;
    }
    if (this->attenuation_linear != other.attenuation_linear) {
      return false;
    }
    if (this->attenuation_quadratic != other.attenuation_quadratic) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    if (this->cast_shadows != other.cast_shadows) {
      return false;
    }
    if (this->spot_inner_angle != other.spot_inner_angle) {
      return false;
    }
    if (this->spot_outer_angle != other.spot_outer_angle) {
      return false;
    }
    if (this->spot_falloff != other.spot_falloff) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->parent_id != other.parent_id) {
      return false;
    }
    if (this->intensity != other.intensity) {
      return false;
    }
    return true;
  }
  bool operator!=(const Light_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Light_

// alias to use template instance with default allocator
using Light =
  ros_ign_interfaces::msg::Light_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t Light_<ContainerAllocator>::POINT;
template<typename ContainerAllocator>
constexpr uint8_t Light_<ContainerAllocator>::SPOT;
template<typename ContainerAllocator>
constexpr uint8_t Light_<ContainerAllocator>::DIRECTIONAL;

}  // namespace msg

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__DETAIL__LIGHT__STRUCT_HPP_
