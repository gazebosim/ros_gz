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

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

// include ROS 2
#include <rclcpp/rclcpp.hpp>

#include "bridge.hpp"

// Direction of bridge.
enum Direction
{
  // Both directions.
  BIDIRECTIONAL = 0,
  // Only from IGN to ROS
  FROM_IGN_TO_ROS = 1,
  // Only from ROS to IGN
  FROM_ROS_TO_IGN = 2,
  // Unspecified, only used for services
  DIR_UNSPECIFIED = 3,
};

//////////////////////////////////////////////////
void usage()
{
  std::cout << "Bridge a collection of ROS2 and Ignition Transport topics and services.\n\n" <<
    "  parameter_bridge [<topic@ROS2_type@Ign_type> ..] " <<
    " [<service@ROS2_srv_type[@Ign_req_type@Ign_rep_type]> ..]\n\n" <<
    "Topics: The first @ symbol delimits the topic name from the message types.\n" <<
    "Following the first @ symbol is the ROS message type.\n" <<
    "The ROS message type is followed by an @, [, or ] symbol where\n" <<
    "    @  == a bidirectional bridge, \n" <<
    "    [  == a bridge from Ignition to ROS,\n" <<
    "    ]  == a bridge from ROS to Ignition.\n" <<
    "Following the direction symbol is the Ignition Transport message " <<
    "type.\n\n" <<
    "Services: The first @ symbol delimits the service name from the types.\n" <<
    "Following the first @ symbol is the ROS service type.\n" <<
    "Optionally, you can include the Ignition request and response type\n" <<
    "separated by the @ symbol.\n" <<
    "It is only supported to expose Ignition servces as ROS services, i.e.\n"
    "the ROS service will forward request to the Ignition service and then forward\n"
    "the response back to the ROS client.\n\n"
    "A bidirectional bridge example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String@ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from Ignition to ROS example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String[ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from ROS to Ignition example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String]ignition.msgs" <<
    ".StringMsg\n" <<
    "A service bridge:\n" <<
    "    parameter_bridge /world/default/control@ros_ign_interfaces/srv/ControlWorld\n" <<
    "Or equivalently:\n" <<
    "    parameter_bridge /world/default/control@ros_ign_interfaces/srv/ControlWorld@"
    "ignition.msgs.WorldControl@ignition.msgs.Boolean\n" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }
  // skip the process name in argument procesing
  ++argv;
  --argc;
  auto filteredArgs = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // ROS 2 node
  auto ros_node = std::make_shared<rclcpp::Node>("ros_ign_bridge");

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::vector<ros_ign_bridge::BridgeHandles> bidirectional_handles;
  std::vector<ros_ign_bridge::BridgeIgnToRosHandles> ign_to_ros_handles;
  std::vector<ros_ign_bridge::BridgeRosToIgnHandles> ros_to_ign_handles;
  std::vector<ros_ign_bridge::BridgeIgnServicesToRosHandles> service_bridge_handles;

  // Filter arguments (i.e. remove ros args) then parse all the remaining ones
  const std::string delim = "@";
  const size_t queue_size = 10;
  // TODO(ivanpauno): Improve the parsing code later, it's hard to read ...
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
    Direction direction = BIDIRECTIONAL;
    if (delimPos == std::string::npos || delimPos == 0) {
      delimPos = arg.find("[");
      if (delimPos == std::string::npos || delimPos == 0) {
        delimPos = arg.find("]");
        if (delimPos == 0) {
          usage();
          return -1;
        } else if (delimPos == std::string::npos) {
          direction = DIR_UNSPECIFIED;
        } else {
          direction = FROM_ROS_TO_IGN;
        }
      } else {
        direction = FROM_IGN_TO_ROS;
      }
    }
    std::string ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());
    if (ros_type_name.find("/srv/") != std::string::npos) {
      std::string ign_req_type_name;
      std::string ign_rep_type_name;
      if (direction != DIR_UNSPECIFIED && direction != BIDIRECTIONAL) {
        usage();
        return -1;
      }
      if (direction == BIDIRECTIONAL) {
        delimPos = arg.find("@");
        if (delimPos == std::string::npos || delimPos == 0) {
          usage();
          return -1;
        }
        ign_req_type_name = arg.substr(0, delimPos);
        arg.erase(0, delimPos + delim.size());
        ign_rep_type_name = std::move(arg);
      }
      try {
        service_bridge_handles.push_back(
          ros_ign_bridge::create_service_bridge(
            ros_node,
            ign_node,
            ros_type_name,
            ign_req_type_name,
            ign_rep_type_name,
            topic_name));
      } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
      }
      continue;
    }

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty()) {
      usage();
      return -1;
    }
    std::string ign_type_name = arg;
    try {
      switch (direction) {
        default:
        case BIDIRECTIONAL:
          bidirectional_handles.push_back(
            ros_ign_bridge::create_bidirectional_bridge(
              ros_node, ign_node,
              ros_type_name, ign_type_name,
              topic_name, queue_size));
          break;
        case FROM_IGN_TO_ROS:
          ign_to_ros_handles.push_back(
            ros_ign_bridge::create_bridge_from_ign_to_ros(
              ign_node, ros_node,
              ign_type_name, topic_name, queue_size,
              ros_type_name, topic_name, queue_size));
          break;
        case FROM_ROS_TO_IGN:
          ros_to_ign_handles.push_back(
            ros_ign_bridge::create_bridge_from_ros_to_ign(
              ros_node, ign_node,
              ros_type_name, topic_name, queue_size,
              ign_type_name, topic_name, queue_size));
          break;
      }
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
