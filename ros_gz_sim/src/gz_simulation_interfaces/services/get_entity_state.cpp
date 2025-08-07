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
#include <string>

#include <gz/sim/Util.hh>

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

GetEntityState::GetEntityState(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  auto service_cb = [this](RequestPtr request, ResponsePtr response) {
    this->gz_proxy_->WithEcm(
      [&](const auto & ecm) { GetEntityState::FromEcm(ecm, request->entity, response->state); });
  };
  this->services_handle_ =
    ros_node->create_service<GetEntityStateSrv>("get_entity_state", service_cb);

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "get_entity_state");
}

Result GetEntityState::FromEcm(
  const gz::sim::EntityComponentManager & ecm, const std::string & name,
  simulation_interfaces::msg::EntityState & state)
{
  Result result;
  auto entity = ecm.EntityByName(name);
  if (entity) {
    return GetEntityState::FromEcm(ecm, *entity, state);
  } else {
    result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
    result.error_message = "Requested entity not found";
    return result;
  }
}

simulation_interfaces::msg::Result GetEntityState::FromEcm(
  const gz::sim::EntityComponentManager & ecm, const gz::sim::Entity & entity,
  simulation_interfaces::msg::EntityState & state)
{
  Result result;
  auto gz_pose = gz::sim::worldPose(entity, ecm);
  ConvertPose(gz_pose, state.pose);
  result.result = simulation_interfaces::msg::Result::RESULT_OK;
  return result;
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
