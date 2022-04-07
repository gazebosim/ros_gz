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

#include <rclcpp/rclcpp.hpp>
#include <ros_ign_bridge/ros_ign_bridge.hpp>

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

//////////////////////////////////////////////////
void usage()
{
  std::cout << "Bridge a collection of ROS2 and Ignition Transport topics.\n\n" <<
    "  parameter_bridge <topic@ROS2_type@Ign_type> .. " <<
    " <topic@ROS2_type@Ign_type>\n\n" <<
    "The first @ symbol delimits the topic name from the message types.\n" <<
    "Following the first @ symbol is the ROS message type.\n" <<
    "The ROS message type is followed by an @, [, or ] symbol where\n" <<
    "    @  == a bidirectional bridge, \n" <<
    "    [  == a bridge from Ignition to ROS,\n" <<
    "    ]  == a bridge from ROS to Ignition.\n" <<
    "Following the direction symbol is the Ignition Transport message " <<
    "type.\n\n" <<
    "A bidirectional bridge example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String@ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from Ignition to ROS example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String[ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from ROS to Ignition example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String]ignition.msgs" <<
    ".StringMsg" << std::endl;
}

//////////////////////////////////////////////////
std::vector<std::string> filter_args(int argc, char * argv[])
{
  const std::string rosArgsBeginDelim = "--ros-args";
  const std::string rosArgsEndDelim = "--";
  // Skip first argument (executable path)
  std::vector<std::string> args(argv + 1, argv + argc);
  auto rosArgsPos = std::find(args.begin(), args.end(), rosArgsBeginDelim);
  auto rosArgsEndPos = std::find(rosArgsPos, args.end(), rosArgsEndDelim);
  // If -- was found, delete it as well
  if (rosArgsEndPos != args.end()) {
    ++rosArgsEndPos;
  }
  // Delete args between --ros-args and -- (or --ros-args to end if not found)
  if (rosArgsPos != args.end()) {
    args.erase(rosArgsPos, rosArgsEndPos);
  }
  return args;
}

using RosIgnBridge = ros_ign_bridge::RosIgnBridge;
using BridgeDirection = ros_ign_bridge::BridgeDirection;
using BridgeConfig = ros_ign_bridge::BridgeConfig;

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }

  rclcpp::init(argc, argv);

  auto bridge_node = std::make_shared<RosIgnBridge>(rclcpp::NodeOptions());

  // Set lazy subscriber on a global basis
  bridge_node->declare_parameter<bool>("lazy", false);
  bool lazy_subscription;
  bridge_node->get_parameter("lazy", lazy_subscription);

  // Filter arguments (i.e. remove ros args) then parse all the remaining ones
  const std::string delim = "@";
  auto filteredArgs = filter_args(argc, argv);
  for (auto & arg : filteredArgs) {
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0) {
      usage();
      return -1;
    }

    BridgeConfig config;
    config.ros_topic_name = arg.substr(0, delimPos);
    config.ign_topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   [ == only from IGN to ROS, or
    //   ] == only from ROS to IGN.
    delimPos = arg.find("@");

    config.direction = BridgeDirection::BIDIRECTIONAL;

    if (delimPos == std::string::npos || delimPos == 0) {
      delimPos = arg.find("[");
      if (delimPos == std::string::npos || delimPos == 0) {
        delimPos = arg.find("]");
        if (delimPos == std::string::npos || delimPos == 0) {
          usage();
          return -1;
        } else {
          config.direction = BridgeDirection::ROS_TO_IGN;
        }
      } else {
        config.direction = BridgeDirection::IGN_TO_ROS;
      }
    }
    config.ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty()) {
      usage();
      return -1;
    }
    config.ign_type_name = arg;
    bridge_node->add_bridge(config);
  }

  // ROS 2 spinner
  rclcpp::spin(bridge_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
