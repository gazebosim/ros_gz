// generated from ros_ign_bridge/resource/get_mappings.cpp.em

@###############################################
@#
@# Methods for determing mappings between
@# Ignition and ROS interfaces
@#
@# EmPy template for generating get_mappings.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - mappings (list of ros_ign_bridge.Mapping)
@#    Mapping between messages as well as their fields
@###############################################
@
#include <map>
#include <string>

#include "get_mappings.hpp"

namespace ros_ign_bridge
{

bool
get_ign_to_ros_mapping(const std::string & ign_type_name, std::string & ros_type_name)
{
@[if not mappings]@
  (void)ign_type_name;
  (void)ros_type_name;
@[end if]@

@[for m in mappings]@
  if (ign_type_name == "@(m.ign_string())")
  {
    ros_type_name = "@(m.ros2_string())";
    return true;
  }
@[end for]@

  return false;
}

bool
get_ros_to_ign_mapping(const std::string & ros_type_name, std::string & ign_type_name)
{
@[if not mappings]@
  (void)ign_type_name;
  (void)ros_type_name;
@[end if]@

@[for m in mappings]@
  if (ros_type_name == "@(m.ros2_string())")
  {
    ign_type_name = "@(m.ign_string())";
    return true;
  }
@[end for]@

  return false;
}

std::multimap<std::string, std::string>
get_all_message_mappings_ros_to_ign()
{
  static std::multimap<std::string, std::string> mappings = {
@[for m in mappings]@
    {
      "@(m.ros2_string())",  // ROS 2
      "@(m.ign_string())", // Ignition
    },
@[end for]@
  };
  return mappings;
}

}  // namespace ros_ign_bridge
