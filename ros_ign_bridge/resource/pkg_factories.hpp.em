// generated from ros_ign_bridge/resource/pkg_factories.hpp.em

@###############################################
@#
@# Factory template specializations based on
@# message types of a single ROS 2 package
@#
@# EmPy template for generating factories/<pkgname>.hpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_name (str)
@#    The ROS 2 package name of this file
@###############################################
@

#ifndef FACTORIES_@(ros2_package_name.upper())
#define FACTORIES_@(ros2_package_name.upper())

#include <memory>
#include <string>

#include "factory_interface.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__@(ros2_package_name)(
  const std::string & ros_type_name, 
  const std::string & ign_type_name);

}  // namespace ros_ign_bridge

#endif  // FACTORIES_@(ros2_package_name.upper())
