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

#include <iostream>
#include <list>
#include <memory>
#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <ros/console.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include "ros1_ign_bridge/bridge.hpp"

// Direction of bridge.
enum Direction
{
  // Both directions.
  BIDIRECTIONAL = 0,
  // Only from IGN to ROS
  FROM_IGN_TO_ROS = 1,
  // Only from ROS to IGN
  FROM_ROS_TO_IGN = 2,
};

//////////////////////////////////////////////////
void usage()
{
  ROS_ERROR_STREAM(
      "Bridge a collection of ROS1 and Ignition Transport topics.\n\n"
      << "  parameter_bridge <topic@ROS1_type[@,<,>]Ign_type> .. "
      << " <topic@ROS1_type@Ign_type>\n\n"
      << "The first @ symbol delimits the topic name from the message types.\n"
      << "Following the first @ symbol is the ROS1 message type.\n"
      << "The ROS1 message type is followed by an @, <, or > symbol where\n"
      << "    @  ==  a bidirectional bridge, \n"
      << "    <  == a bridge from Ignition to ROS1,\n"
      << "    >  == a bridge from ROS1 to Ignition.\n"
      << "Following the direction symbol is the Ignition Transport message "
      << "type.\n"
      << "A bidirectional bridge example:\n"
      << "    parameter_bridge /chatter@std_msgs/String@ignition.msgs"
      << ".StringMsg" << std::endl);
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2)
  {
    usage();
    return -1;
  }

  // ROS 1 node
  ros::init(argc, argv, "ros_ign_bridge");
  ros::NodeHandle ros1_node;

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::list<ros1_ign_bridge::BridgeHandles> bidirectional_handles;
  std::list<ros1_ign_bridge::BridgeIgnto1Handles> ign_to_ros1_handles;
  std::list<ros1_ign_bridge::Bridge1toIgnHandles> ros1_to_ign_handles;

  // Parse all arguments.
  const std::string delim = "@";
  const size_t queue_size = 10;
  for (auto i = 1; i < argc; ++i)
  {
    std::string arg = std::string(argv[i]);
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0)
    {
     usage();
     return -1;
    }
    std::string topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   < == only from IGN to ROS, or
    //   > == only from ROS to IGN.
    delimPos = arg.find("@");
    Direction direction = BIDIRECTIONAL;
    if (delimPos == std::string::npos || delimPos == 0)
    {
      delimPos = arg.find("<");
      if (delimPos == std::string::npos || delimPos == 0)
      {
        delimPos = arg.find(">");
        if (delimPos == std::string::npos || delimPos == 0)
        {
          usage();
          return -1;
        }
        else
        {
          direction = FROM_ROS_TO_IGN;
        }
      }
      else
      {
        direction = FROM_IGN_TO_ROS;
      }
    }
    std::string ros1_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty())
    {
      usage();
      return -1;
    }
    std::string ign_type_name = arg;

    try
    {
      switch (direction)
      {
        default:
        case BIDIRECTIONAL:
          bidirectional_handles.push_back(
              ros1_ign_bridge::create_bidirectional_bridge(
                ros1_node, ign_node,
                ros1_type_name, ign_type_name,
                topic_name, queue_size));
          break;
        case FROM_IGN_TO_ROS:
          ign_to_ros1_handles.push_back(
              ros1_ign_bridge::create_bridge_from_ign_to_ros(
                ign_node, ros1_node,
                ign_type_name, topic_name, queue_size,
                ros1_type_name, topic_name, queue_size));
          break;
        case FROM_ROS_TO_IGN:
          ros1_to_ign_handles.push_back(
              ros1_ign_bridge::create_bridge_from_ros_to_ign(
                ros1_node, ign_node,
                ros1_type_name, topic_name, queue_size,
                ign_type_name, topic_name, queue_size));
          break;
      }
    }
    catch (std::runtime_error &_e)
    {
      ROS_ERROR_STREAM("Failed to create a bridge for topic ["
          << topic_name << "] "
          << "with ROS1 type [" << ros1_type_name << "] and "
          << "Ignition Transport type [" << ign_type_name << "]"
          << std::endl);
    }
  }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
