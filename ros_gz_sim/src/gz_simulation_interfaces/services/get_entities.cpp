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

#include "get_entities.hpp"

#include <gz/msgs/boolean.pb.h>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "../gazebo_state.hpp"
#include "simulation_interfaces/srv/get_entities.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetEntitiesSrv = simulation_interfaces::srv::GetEntities;
using RequestPtr = GetEntitiesSrv::Request::ConstSharedPtr;
using ResponsePtr = GetEntitiesSrv::Response::SharedPtr;

GetEntities::GetEntities(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboState> gz_state)
: HandlerBase(ros_node, gz_state)
{
  this->services_handle_ = ros_node->create_service<GetEntitiesSrv>(
    "get_entities", [this](RequestPtr request, ResponsePtr response) {
      this->gz_state_->Each<components::Name, components::Model>(
        [&](const gz::sim::Entity &, const components::Name * name, const components::Model *) {
          response->entities.push_back(name->Data());
          return true;
        });

      // TODO(azeey) Ensure that entities listed are top level models
      // TODO(azeey) Implement filtering by name
      // TODO(azeey) Implement filtering by category
      // TODO(azeey) Implement filtering by bounds
      // TODO(azeey) Implement error checking and setting error message
    });

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "get_entities");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
