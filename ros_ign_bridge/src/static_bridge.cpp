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

#include <memory>
#include <string>

// include Ignition Transport
#include <ignition/transport/Node.hh>

// include ROS 2
#include <rclcpp/rclcpp.hpp>

#include "bridge.hpp"
#include "service_factory.hpp"

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  // ROS node
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("test_node");

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  // bridge one example topic
  std::string topic_name = "chatter";
  std::string ros_type_name = "std_msgs/msg/String";
  std::string ign_type_name = "ignition.msgs.StringMsg";
  size_t queue_size = 10;

  auto handles = ros_ign_bridge::create_bidirectional_bridge(
    ros_node, ign_node, ros_type_name, ign_type_name, topic_name, queue_size);

  rclcpp::spin(ros_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
