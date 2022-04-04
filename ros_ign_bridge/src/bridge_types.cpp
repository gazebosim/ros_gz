// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "get_mappings.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

bool find_command_option(const std::vector<std::string> & args, const std::string & option)
{
  return std::find(args.begin(), args.end(), option) != args.end();
}

bool get_flag_option(const std::vector<std::string> & args, const std::string & option)
{
  auto it = std::find(args.begin(), args.end(), option);
  return it != args.end();
}

int main(int argc, char * argv[])
{
  std::vector<std::string> args(argv, argv + argc);

  if (find_command_option(args, "-h") || find_command_option(args, "--help")) {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << " -h, --help: This message." << std::endl;
    ss << " --print-pairs: Print a list of the supported ROS 2 <=> IGN conversion pairs.";
    ss << std::endl;
    ss << " --print-ign: Print a list of the supported Ignition types.";
    ss << std::endl;
    ss << " --print-ros: Print a list of the supported ROS 2 types.";
    ss << std::endl;
    ss << " --get-ign-to-ros <MSG>: Get the ROS 2 mapping for an Ignition Type.";
    ss << std::endl;
    ss << " --get-ros-to-ign <MSG>: Get the Ignition mapping for a ROS 2 Type.";
    std::cout << ss.str();
    return 0;
  }

  if (get_flag_option(args, "--get-ros-to-ign"))
  {
    if (args.size() < 3)
    {
      printf("--get-ros-to-ign requires an argument\n");
      return 1;
    }

    std::string ign_type_name;
    auto found = ros_ign_bridge::get_ros_to_ign_mapping(args[2], ign_type_name);
    if (found)
    {
      printf("%s\n", ign_type_name.c_str());
      return 0;
    } else {
      return 1;
    }
  }
  else if (get_flag_option(args, "--get-ign-to-ros"))
  {
    if (args.size() < 3)
    {
      printf("--get-ign-to-ros requires an argument\n");
      return 1;
    }

    std::string ros_type_name;
    auto found = ros_ign_bridge::get_ign_to_ros_mapping(args[2], ros_type_name);
    if (found)
    {
      printf("%s\n", ros_type_name.c_str());
      return 0;
    } else {
      return 1;
    }
  }
  else if (get_flag_option(args, "--print-ign"))
  {
    auto mappings = ros_ign_bridge::get_all_message_mappings_ros_to_ign();
    if (mappings.size() > 0) {
      for (auto & pair : mappings) {
        printf("%s\n", pair.second.c_str());
      }
    } else {
      printf("No message type conversion pairs supported.\n");
    }
  }
  else if (get_flag_option(args, "--print-ros"))
  {
    auto mappings = ros_ign_bridge::get_all_message_mappings_ros_to_ign();
    if (mappings.size() > 0) {
      for (auto & pair : mappings) {
        printf("%s\n", pair.first.c_str());
      }
    } else {
      printf("No message type conversion pairs supported.\n");
    }
  }
  else if (get_flag_option(args, "--print-pairs")) {
    auto mappings = ros_ign_bridge::get_all_message_mappings_ros_to_ign();
    if (mappings.size() > 0) {
      printf("Supported ROS <=> Ignition  message type conversion pairs:\n");
      for (auto & pair : mappings) {
        printf("  - '%s' (ROS) <=> '%s' (IGN)\n", pair.first.c_str(), pair.second.c_str());
      }
    } else {
      printf("No message type conversion pairs supported.\n");
    }
  }
}
