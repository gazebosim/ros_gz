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

#include "get_entity_info.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/world_control_state.pb.h>

#include <memory>

#include <gz/sim/Server.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SemanticCategory.hh>
#include <gz/sim/components/SemanticDescription.hh>
#include <gz/sim/components/SemanticTags.hh>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/srv/get_entity_info.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetEntityInfoSrv = simulation_interfaces::srv::GetEntityInfo;
using RequestPtr = GetEntityInfoSrv::Request::ConstSharedPtr;
using ResponsePtr = GetEntityInfoSrv::Response::SharedPtr;
using simulation_interfaces::msg::Result;

GetEntityInfo::GetEntityInfo(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  this->services_handle_ = ros_node->create_service<GetEntityInfoSrv>(
    "get_entity_info", [this](RequestPtr request, ResponsePtr response) {
      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }
      this->gz_proxy_->WithEcm([request, response](gz::sim::EntityComponentManager & ecm) {
        auto entity = ecm.EntityByName(request->entity);
        if (entity) {
          auto category = ecm.ComponentData<components::SemanticCategory>(*entity);
          if (category) {
            response->info.category.set__category(*category);
          }

          auto description = ecm.ComponentData<components::SemanticDescription>(*entity);

          if (description) {
            response->info.description = *description;
          }

          auto tags = ecm.ComponentData<components::SemanticTags>(*entity);
          if (tags) {
            response->info.tags = *tags;
          }

          response->result.result = Result::RESULT_OK;
        } else {
          response->result.result = Result::RESULT_OPERATION_FAILED;
          response->result.error_message = "Specified entity was not found";
        }
      });
    });

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
