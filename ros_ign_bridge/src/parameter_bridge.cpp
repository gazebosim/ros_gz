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

// include ROS 2
#include <rclcpp/rclcpp.hpp>

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "bridge_ign_to_ros.hpp"
#include "bridge_ros_to_ign.hpp"

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

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }

  rclcpp::init(argc, argv);

  // ROS 2 node
  auto ros_node = std::make_shared<rclcpp::Node>("ros_ign_bridge");

  ros_node->declare_parameter<bool>("lazy", false);

  bool lazy_subscription;
  ros_node->get_parameter("lazy", lazy_subscription);

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::list<ros_ign_bridge::BridgePtr> handles;

  // Filter arguments (i.e. remove ros args) then parse all the remaining ones
  const std::string delim = "@";
  auto filteredArgs = filter_args(argc, argv);
  for (auto & arg : filteredArgs) {
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0) {
      usage();
      return -1;
    }
    std::string topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   [ == only from IGN to ROS, or
    //   ] == only from ROS to IGN.
    delimPos = arg.find("@");
    bool ign_to_ros = true;
    bool ros_to_ign = true;
    if (delimPos == std::string::npos || delimPos == 0) {
      delimPos = arg.find("[");
      if (delimPos == std::string::npos || delimPos == 0) {
        delimPos = arg.find("]");
        if (delimPos == std::string::npos || delimPos == 0) {
          usage();
          return -1;
        } else {
          ign_to_ros = false;
        }
      } else {
        ros_to_ign = false;
      }
    }
    std::string ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty()) {
      usage();
      return -1;
    }
    std::string ign_type_name = arg;

    try {
      if (ign_to_ros) {
        RCLCPP_INFO(
          ros_node->get_logger(),
          "Creating IGN->ROS Bridge: [%s] (%s -> %s) (Lazy %d): ",
          topic_name.c_str(), ign_type_name.c_str(), ros_type_name.c_str(),
          lazy_subscription);
        handles.push_back(
          std::make_unique<ros_ign_bridge::BridgeIgnToRos>(
            ros_node, ign_node,
            ros_type_name, topic_name,
            ign_type_name, topic_name,
            ros_ign_bridge::Bridge::kDefaultSubscriberQueue,
            ros_ign_bridge::Bridge::kDefaultPublisherQueue,
            lazy_subscription
        ));
      }
      if (ros_to_ign) {
        RCLCPP_INFO(
          ros_node->get_logger(),
          "Creating ROS->IGN Bridge: [%s] (%s -> %s) (Lazy %d): ",
          topic_name.c_str(), ros_type_name.c_str(), ign_type_name.c_str(),
          lazy_subscription);
        handles.push_back(
          std::make_unique<ros_ign_bridge::BridgeRosToIgn>(
            ros_node, ign_node,
            ros_type_name, topic_name,
            ign_type_name, topic_name,
            ros_ign_bridge::Bridge::kDefaultSubscriberQueue,
            ros_ign_bridge::Bridge::kDefaultPublisherQueue,
            lazy_subscription
        ));
      }
    } catch (std::runtime_error & _e) {
      RCLCPP_WARN(
        ros_node->get_logger(),
        "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
        "and Ignition Transport type[%s]",
        topic_name.c_str(),
        ros_type_name.c_str(),
        ign_type_name.c_str());
    }
  }

  for (auto & bridge : handles) {
    bridge->Start();
  }

  auto timer = ros_node->create_wall_timer(
    std::chrono::milliseconds(1000),
    [&handles]() {
      for (auto & bridge : handles) {
        bridge->Spin();
      }
    });

  // ROS 2 spinner
  rclcpp::spin(ros_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
