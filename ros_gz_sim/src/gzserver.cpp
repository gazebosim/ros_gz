// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/ServerConfig.hh>
#include <rclcpp/rclcpp.hpp>

// ROS node that executes a gz-sim Server given a world SDF file or string.
int main(int _argc, char ** _argv)
{
  auto filtered_arguments = rclcpp::init_and_remove_ros_arguments(_argc, _argv);
  auto node = rclcpp::Node::make_shared("gzserver");
  auto world_sdf_file = node->declare_parameter("world_sdf_file", "");
  auto world_sdf_string = node->declare_parameter("world_sdf_string", "");

  gz::common::Console::SetVerbosity(4);
  gz::sim::ServerConfig server_config;

  if (!world_sdf_file.empty()) {
    server_config.SetSdfFile(world_sdf_file);
  } else if (!world_sdf_string.empty()) {
    server_config.SetSdfString(world_sdf_string);
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "Must specify either 'world_sdf_file' or 'world_sdf_string'");
    return -1;
  }

  gz::sim::Server server(server_config);
  server.Run(true /*blocking*/, 0, false /*paused*/);
}
