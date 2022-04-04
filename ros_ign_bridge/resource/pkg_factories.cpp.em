// generated from ros_ign_bridge/resource/pkg_factories.cpp.em

@###############################################
@#
@# Factory template specializations based on
@# message types of a single ROS 2 package
@#
@# EmPy template for generating factories/<pkgname>.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_name (str)
@#    The ROS 2 package name of this file
@###############################################
@

#include "factories/@(ros2_package_name).hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_ign_bridge/convert/@(ros2_package_name).hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__@(ros2_package_name)(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
@[for m in mappings]@
  if ((ros_type_name == "@(m.ros2_string())" || ros_type_name.empty()) &&
    ign_type_name == "@(m.ign_string())")
  {
    return std::make_shared<
      Factory<
        @(m.ros2_type()),
        @(m.ign_type())
      >
    >("@(m.ros2_string())", "@(m.ign_string())");
  }
@[end for]@
  return nullptr;
}

@[for m in mappings]@
template<>
void
Factory<
  @(m.ros2_type()),
  @(m.ign_type())
>::convert_ros_to_ign(
  const @(m.ros2_type()) & ros_msg,
  @(m.ign_type()) & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  @(m.ros2_type()),
  @(m.ign_type())
>::convert_ign_to_ros(
  const @(m.ign_type()) & ign_msg,
  @(m.ros2_type()) & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}
@[end for]@

}  // namespace ros_ign_bridge
