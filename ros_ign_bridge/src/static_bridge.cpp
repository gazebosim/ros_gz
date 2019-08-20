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
#include <memory>
#include <string>

// include ROS
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include "ros_ign_bridge/bridge.hpp"

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  // ROS node
  ros::init(argc, argv, "ros_ign_bridge");
  ros::NodeHandle ros_node;

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  // bridge one example topic
  std::string topic_name = "chatter";
  std::string ros_type_name = "std_msgs/String";
  std::string ign_type_name = "ignition.msgs.StringMsg";
  size_t queue_size = 10;

  auto handles = ros_ign_bridge::create_bidirectional_bridge(
    ros_node, ign_node, ros_type_name, ign_type_name, topic_name, queue_size);

  // ROS asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
