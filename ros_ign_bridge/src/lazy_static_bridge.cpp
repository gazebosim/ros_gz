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

// include ROS 2
#include <rclcpp/rclcpp.hpp>

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include <list>
#include <memory>
#include <string>

#include "bridge_ign_to_ros.hpp"
#include "bridge_ros_to_ign.hpp"

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

  std::list<ros_ign_bridge::BridgePtr> handles;

  handles.push_back(
    std::make_unique<ros_ign_bridge::BridgeRosToIgn>(
      ros_node, ign_node,
      ros_type_name, topic_name,
      ign_type_name, topic_name,
      ros_ign_bridge::Bridge::kDefaultSubscriberQueue,
      ros_ign_bridge::Bridge::kDefaultPublisherQueue,
      true));

  handles.push_back(
    std::make_unique<ros_ign_bridge::BridgeIgnToRos>(
      ros_node, ign_node,
      ros_type_name, topic_name,
      ign_type_name, topic_name,
      ros_ign_bridge::Bridge::kDefaultSubscriberQueue,
      ros_ign_bridge::Bridge::kDefaultPublisherQueue,
      true));


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

  rclcpp::spin(ros_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
