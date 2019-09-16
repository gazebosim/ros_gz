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

#include <ros_ign_bridge/bridge.hpp>

#include <iostream>
#include <list>
#include <memory>
#include <string>

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Bridge a collection of ROS2 and Ignition Transport topics.\n\n" <<
    "  parameter_bridge <topic@ROS2_type@Ign_type> .. " <<
    " <topic@ROS2_type@Ign_type>\n\n" <<
    "E.g.: parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs" <<
    ".StringMsg" << std::endl;
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

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::list<ros_ign_bridge::BridgeHandles> all_handles;

  // Parse all arguments.
  const std::string delim = "@";
  const size_t queue_size = 10;
  for (auto i = 1; i < argc; ++i) {
    std::string arg = std::string(argv[i]);
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0) {
      usage();
      return -1;
    }
    std::string topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0) {
      usage();
      return -1;
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
      ros_ign_bridge::BridgeHandles handles =
        ros_ign_bridge::create_bidirectional_bridge(
          ros_node, ign_node,
          ros_type_name, ign_type_name,
          topic_name, queue_size);
      all_handles.push_back(handles);
    } catch (std::runtime_error & _e) {
      std::cerr << "Failed to create a bridge for topic [" << topic_name << "] " <<
        "with ROS2 type [" << ros_type_name << "] and " <<
        "Ignition Transport type [" << ign_type_name << "]" << std::endl;
    }
  }

  // ROS 2 spinner
  rclcpp::spin(ros_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
