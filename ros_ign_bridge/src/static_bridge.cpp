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

#include <memory>
#include <string>

using RosIgnBridge = ros_ign_bridge::RosIgnBridge;

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  // ROS node
  rclcpp::init(argc, argv);
  auto bridge_node = std::make_shared<RosIgnBridge>(rclcpp::NodeOptions());

  // Set lazy subscriber on a global basis
  bridge_node->declare_parameter<bool>("lazy", false);
  bool lazy_subscription;
  bridge_node->get_parameter("lazy", lazy_subscription);

  // bridge one example topic
  ros_ign_bridge::BridgeConfig config;
  config.ros_topic_name = "chatter";
  config.ign_topic_name = "chatter";
  config.ros_type_name = "std_msgs/msg/String";
  config.ign_type_name = "ignition.msgs.StringMsg";
  config.is_lazy = lazy_subscription;

  bridge_node->add_bridge(config);

  rclcpp::spin(bridge_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
