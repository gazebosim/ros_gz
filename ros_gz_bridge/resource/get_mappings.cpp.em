// generated from ros_gz_bridge/resource/get_mappings.cpp.em

@###############################################
@#
@# Methods for determing mappings between
@# Gazebo and ROS interfaces
@#
@# EmPy template for generating get_mappings.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - mappings (list of ros_gz_bridge.Mapping)
@#    Mapping between messages as well as their fields
@###############################################
@
#include <map>
#include <string>

#include "get_mappings.hpp"

namespace ros_gz_bridge
{

bool
get_gz_to_ros_mapping(const std::string & gz_type_name, std::string & ros_type_name)
{
@[if not mappings]@
  (void)gz_type_name;
  (void)ros_type_name;
@[end if]@

@[for m in mappings]@
  if (gz_type_name == "@(m.gz_string())" || gz_type_name == "@(m.ign_string())")
  {
    ros_type_name = "@(m.ros2_string())";
    return true;
  }
@[end for]@

  return false;
}

bool
get_ros_to_gz_mapping(const std::string & ros_type_name, std::string & gz_type_name)
{
@[if not mappings]@
  (void)gz_type_name;
  (void)ros_type_name;
@[end if]@

@[for m in mappings]@
  if (ros_type_name == "@(m.ros2_string())")
  {
    gz_type_name = "@(m.gz_string())";
    return true;
  }
@[end for]@

  return false;
}

std::multimap<std::string, std::string>
get_all_message_mappings_ros_to_gz()
{
  static std::multimap<std::string, std::string> mappings = {
@[for m in mappings]@
    {
      "@(m.ros2_string())",  // ROS 2
      "@(m.gz_string())", // Gazebo
    },
    {
      "@(m.ros2_string())",  // ROS 2
      "@(m.ign_string())", // Gazebo
    },
@[end for]@
  };
  return mappings;
}

}  // namespace ros_gz_bridge
