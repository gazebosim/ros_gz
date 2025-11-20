// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include "get_simulator_features.hpp"

#include <gz/msgs/boolean.pb.h>

#include <memory>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/srv/get_simulator_features.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetSimulatorFeaturesSrv = simulation_interfaces::srv::GetSimulatorFeatures;
using RequestPtr = GetSimulatorFeaturesSrv::Request::ConstSharedPtr;
using ResponsePtr = GetSimulatorFeaturesSrv::Response::SharedPtr;

GetSimulatorFeatures::GetSimulatorFeatures(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  this->services_handle_ = ros_node->create_service<GetSimulatorFeaturesSrv>(
    "get_simulator_features", [](RequestPtr, ResponsePtr response) {
      using SimulatorFeatures = simulation_interfaces::msg::SimulatorFeatures;
      // clang-format off
      response->features.features.assign({
            SimulatorFeatures::SPAWNING,
            SimulatorFeatures::DELETING,
            SimulatorFeatures::ENTITY_TAGS,
            // SimulatorFeatures::ENTITY_BOUNDS, // TODO(azeey)
            // SimulatorFeatures::ENTITY_BOUNDS_BOX, // TODO(azeey)
            SimulatorFeatures::ENTITY_CATEGORIES,
            SimulatorFeatures::SPAWNING_RESOURCE_STRING,
            SimulatorFeatures::ENTITY_STATE_GETTING,
            SimulatorFeatures::ENTITY_STATE_SETTING,
            SimulatorFeatures::ENTITY_INFO_GETTING,
            // SimulatorFeatures::ENTITY_INFO_SETTING, // TODO(azeey)
            SimulatorFeatures::SIMULATION_RESET,
            // SimulatorFeatures::SIMULATION_RESET_TIME, // TODO(azeey)
            // SimulatorFeatures::SIMULATION_RESET_STATE, // TODO(azeey)
            // SimulatorFeatures::SIMULATION_RESET_SPAWNED, // TODO(azeey)
            SimulatorFeatures::SIMULATION_STATE_GETTING,
            SimulatorFeatures::SIMULATION_STATE_SETTING,
            SimulatorFeatures::SIMULATION_STATE_PAUSE,
            SimulatorFeatures::STEP_SIMULATION_SINGLE,
            SimulatorFeatures::STEP_SIMULATION_MULTIPLE,
            SimulatorFeatures::STEP_SIMULATION_ACTION,
        // clang-format on
      });

      response->features.spawn_formats.assign({"sdf", "urdf"});
      // TODO(azeey) Fill in custom_info
    });

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
