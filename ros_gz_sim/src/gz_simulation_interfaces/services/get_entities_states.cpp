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

#include <memory>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <simulation_interfaces/srv/get_entities_states.hpp>

#include "../gazebo_proxy.hpp"
#include "../gz_entity_filters.hpp"
#include "get_entity_state.hpp"

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
      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }
      const auto stats = this->gz_proxy_->Stats();
      this->gz_proxy_->WithEcm([&](const gz::sim::EntityComponentManager & ecm) {
          try {
            // Since we will be entering the `Each` loop, we start with RESULT_OK and override it
            // with any non-okay result in the loop. The loop breaks by return false if any result
            // other than RESULT_OK is encountered.
            response->result.result = Result::RESULT_OK;
            GzEntityFilters filters(request->filters, ecm);
            ecm.Each<components::Name, components::Model, components::ParentEntity>(
              [&](
                const gz::sim::Entity & entity, const components::Name * name,
                const components::Model *, const components::ParentEntity * parent) {
                // Check that this is a top level model
                if (ecm.Component<components::Model>(parent->Data())) {
                  // This is a nested model which should not be included in the list of entities to
                  // return.
                  // TODO(azeey) It might be useful to allow nested models here when we enable
                  // setting their poses in Gazebo.
                  return true;
                }
                auto [isIncluded, filter_result] = filters.ApplyFilter(entity, name->Data());

                if (filter_result.result != Result::RESULT_OK) {
                  response->result = filter_result;
                  return false;
                }

                if (!isIncluded) {
                  return true;
                }

                response->entities.push_back(name->Data());
                auto & state = response->states.emplace_back();
                auto state_result = GetEntityState::FromEcm(ecm, stats, entity, state);

                if (state_result.result != Result::RESULT_OK) {
                  response->result = state_result;
                  return false;
                }

                return true;
            });
          } catch (const std::exception & e) {
            response->result.result = Result::RESULT_OPERATION_FAILED;
            response->result.error_message = e.what();
          }
    });
    };

  this->services_handle_ =
    ros_node->create_service<GetEntitiesStatesSrv>("get_entities_states", service_cb);

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
