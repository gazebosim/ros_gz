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

#include "ros_gz_sim/gzserver.hpp"

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/SystemLoader.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "ros_gz_sim/gz_simulation_interfaces.hpp"

namespace ros_gz_sim
{

class GzServer::Implementation
{
  /// \brief We don't want to block the ROS thread.

public:
  std::thread thread;
  std::unique_ptr<gz_simulation_interfaces::GzSimulationInterfaces> sim_interfaces;
};

GzServer::GzServer(const rclcpp::NodeOptions & options)
: Node("gzserver", options), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  rclcpp::uninstall_signal_handlers();
  this->dataPtr->thread = std::thread(std::bind(&GzServer::OnStart, this));
}

GzServer::~GzServer()
{
  // Make sure to join the thread on shutdown.
  if (this->dataPtr->thread.joinable()) {
    this->dataPtr->thread.join();
  }
}

void GzServer::OnStart()
{
  auto world_sdf_file = this->declare_parameter("world_sdf_file", "");
  auto world_sdf_string = this->declare_parameter("world_sdf_string", "");
  auto initial_sim_time = this->declare_parameter("initial_sim_time", 0.0);
  const auto verbosity_level = this->declare_parameter("verbosity_level", 4);

  gz::common::Console::SetVerbosity(verbosity_level);
  gz::sim::ServerConfig server_config;

  if (!world_sdf_file.empty()) {
    server_config.SetSdfFile(world_sdf_file);
  } else if (!world_sdf_string.empty()) {
    server_config.SetSdfString(world_sdf_string);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Must specify either 'world_sdf_file' or 'world_sdf_string'");
    rclcpp::shutdown();
    return;
  }
  server_config.SetInitialSimTime(initial_sim_time);

  auto server = std::make_unique<gz::sim::Server>(server_config);
  // TODO(azeey) Think about whether it makes sense to RunOnce paused here or wait for some
  // critical services from Gazebo to become available before starting the ROS services
  server->RunOnce(true);
  // TODO(azeey) Allow disabling simulation interfaces

  this->dataPtr->sim_interfaces =
    std::make_unique<gz_simulation_interfaces::GzSimulationInterfaces>(
      this->create_sub_node(this->get_name()));
  server->Run(true /*blocking*/, 0, false /*paused*/);
  server.reset();
  // Call shutdown before resetting sim_interfaces so that threads in sim_interfaces can gracefully
  // exit before being destructed.
  rclcpp::shutdown();
  this->dataPtr->sim_interfaces.reset();
}

}  // namespace ros_gz_sim

RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_sim::GzServer)
