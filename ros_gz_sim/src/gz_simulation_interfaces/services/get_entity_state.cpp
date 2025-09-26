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

#include "get_entity_state.hpp"

#include <gz/msgs/boolean.pb.h>

#include <memory>
#include <optional>
#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include "../gazebo_proxy.hpp"
#include "../utils.hpp"
#include "simulation_interfaces/srv/get_entity_state.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetEntityStateSrv = simulation_interfaces::srv::GetEntityState;
using RequestPtr = GetEntityStateSrv::Request::ConstSharedPtr;
using ResponsePtr = GetEntityStateSrv::Response::SharedPtr;

using simulation_interfaces::msg::Result;
namespace components = gz::sim::components;

GetEntityState::GetEntityState(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  auto service_cb = [this](RequestPtr request, ResponsePtr response) {
      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }
      auto stats = this->gz_proxy_->Stats();
      this->gz_proxy_->WithEcm([&](const auto & ecm) {
          response->result = GetEntityState::FromEcm(ecm, stats, request->entity, response->state);
    });
    };
  this->services_handle_ =
    ros_node->create_service<GetEntityStateSrv>("get_entity_state", service_cb);

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}

Result GetEntityState::FromEcm(
  const gz::sim::EntityComponentManager & ecm, const gz::msgs::WorldStatistics & stats,
  const std::string & name, simulation_interfaces::msg::EntityState & state)
{
  Result result;
  auto entity = ecm.EntityByName(name);
  if (entity) {
    return GetEntityState::FromEcm(ecm, stats, *entity, state);
  } else {
    result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
    result.error_message = "Requested entity not found";
    return result;
  }
}

simulation_interfaces::msg::Result GetEntityState::FromEcm(
  const gz::sim::EntityComponentManager & ecm, const gz::msgs::WorldStatistics & stats,
  const gz::sim::Entity & entity, simulation_interfaces::msg::EntityState & state)
{
  Result result;
  state.header.frame_id = "world";
  state.header.stamp = ConvertTime(stats.sim_time());
  // If the entity is a static model, we will only fill in the pose
  gz::sim::Model model(entity);
  if (model.Static(ecm)) {
    auto pose = gz::sim::worldPose(entity, ecm);
    ConvertPose(pose, state.pose);
    result.result = simulation_interfaces::msg::Result::RESULT_OK;
    return result;
  }
  // Find the canonical link that corresponds to this model
  auto canonicalLinkEntity = model.CanonicalLink(ecm);
  if (canonicalLinkEntity == gz::sim::kNullEntity) {
    // TODO(azeey) Handle this error condition. Maybe a static model
    result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
    result.error_message = "Could not find canonical link";
    return result;
  }
  gz::sim::Link canonicalLink(canonicalLinkEntity);
  // See https://drake.mit.edu/doxygen_cxx/group__multibody__quantities.html for notations
  std::optional<gz::math::Pose3d> X_WL = canonicalLink.WorldPose(ecm);
  auto X_LM = ecm.ComponentData<components::Pose>(canonicalLinkEntity)->Inverse();
  std::optional<gz::math::Vector3d> v_WM = canonicalLink.WorldLinearVelocity(ecm, X_LM.Pos());
  std::optional<gz::math::Vector3d> w_WL = canonicalLink.WorldAngularVelocity(ecm);

  if (X_WL && v_WM && w_WL) {
    // TODO(azeey) Add comments
    // Find the pose of the model based on the pose of the pose of the canonical link.
    // X_ML: pose of the canonical link w.r.t the model
    auto X_WM = (*X_WL) * X_LM;
    ConvertPose(X_WM, state.pose);

    ConvertVector3(*v_WM, state.twist.linear);
    ConvertVector3(*w_WL, state.twist.angular);
    result.result = simulation_interfaces::msg::Result::RESULT_OK;
  } else {
    result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
    // TODO(azeey) Fix error message
    result.error_message = "WorldPose component not set on entity";
  }
  return result;
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
