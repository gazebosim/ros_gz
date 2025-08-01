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

#include "get_entities_states.hpp"

#include <gz/msgs/boolean.pb.h>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "../gazebo_proxy.hpp"
#include "get_entity_state.hpp"
#include "simulation_interfaces/srv/get_entities_states.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{

using GetEntitiesStatesSrv = simulation_interfaces::srv::GetEntitiesStates;
using RequestPtr = GetEntitiesStatesSrv::Request::ConstSharedPtr;
using ResponsePtr = GetEntitiesStatesSrv::Response::SharedPtr;

using simulation_interfaces::msg::Result;

GetEntitiesStates::GetEntitiesStates(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  auto service_cb = [this](RequestPtr request, ResponsePtr response) {
    this->gz_proxy_->WithLockedEcm([&](const gz::sim::EntityComponentManager & ecm) {
      ecm.Each<components::Name, components::Model>([&](
                                                      const gz::sim::Entity & entity,
                                                      const components::Name * name,
                                                      const components::Model *) {
        response->entities.push_back(name->Data());
        auto & state = response->states.emplace_back();
        auto result = GetEntityState::FromEcm(ecm, entity, state);
        response->set__result(result);
        // Continue the Each loop only if the result from previous run is okay.
        return (result.result == Result::RESULT_OK);
      });
    });
  };

  this->services_handle_ =
    ros_node->create_service<GetEntitiesStatesSrv>("get_entities_states", service_cb);

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "get_entities_states");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
