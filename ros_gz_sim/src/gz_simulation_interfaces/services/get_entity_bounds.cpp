// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include "get_entity_bounds.hpp"

#include <memory>
#include <optional>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

#include "../gazebo_proxy.hpp"
#include "../utils.hpp"
#include "simulation_interfaces/msg/bounds.hpp"
#include "simulation_interfaces/srv/get_entity_bounds.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetEntityBoundsSrv = simulation_interfaces::srv::GetEntityBounds;
using RequestPtr = GetEntityBoundsSrv::Request::ConstSharedPtr;
using ResponsePtr = GetEntityBoundsSrv::Response::SharedPtr;

using simulation_interfaces::msg::Bounds;
using simulation_interfaces::msg::Result;

GetEntityBounds::GetEntityBounds(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  auto service_cb = [this](RequestPtr request, ResponsePtr response) {
      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }
      // Mutable ECM: EnableBoundingBoxChecks creates the AxisAlignedBox
      // component that Physics fills.
      this->gz_proxy_->WithEcm([&](gz::sim::EntityComponentManager & ecm) {
          auto entity = ecm.EntityByName(request->entity);
          if (!entity) {
            response->result.result = Result::RESULT_NOT_FOUND;
            response->result.error_message = "Requested entity not found";
            return;
          }
          gz::sim::Model model(*entity);
          auto linkEntity = model.CanonicalLink(ecm);
          if (linkEntity == gz::sim::kNullEntity) {
            response->result.result = Result::RESULT_OPERATION_FAILED;
            response->result.error_message = "Entity has no canonical link";
            return;
          }
          gz::sim::Link link(linkEntity);
          // Gazebo does not report a link's bounding box unless asked. Enabling
          // is idempotent; Physics initialises and then keeps the
          // components::AxisAlignedBox up to date from the collision shapes.
          link.EnableBoundingBoxChecks(ecm, true);
          // AABB in the LINK frame — matches the Bounds contract (bounds are
          // relative to the entity's canonical link transform, REP-103).
          std::optional<gz::math::AxisAlignedBox> aabb = link.AxisAlignedBox(ecm);
          if (!aabb.has_value()) {
            // Just enabled; Physics has not filled the component yet. Callers
            // poll (e.g. the GT perception shim), so report a transient failure
            // and let them retry on the next tick.
            response->result.result = Result::RESULT_OPERATION_FAILED;
            response->result.error_message =
            "Bounding box not available yet — enabled, retry";
            return;
          }
          Bounds bounds;
          bounds.type = Bounds::TYPE_BOX;
          bounds.points.resize(2);
          ConvertVector3(aabb->Min(), bounds.points[0]);
          ConvertVector3(aabb->Max(), bounds.points[1]);
          response->bounds = bounds;
          response->result.result = Result::RESULT_OK;
    });
    };
  this->services_handle_ =
    ros_node->create_service<GetEntityBoundsSrv>("get_entity_bounds", service_cb);

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
