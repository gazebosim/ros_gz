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

GetEntityState::GetEntityState(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  this->services_handle_ = ros_node->create_service<GetEntityStateSrv>(
    "get_entity_state", [this](RequestPtr request, ResponsePtr response) {
      auto gz_state = this->gz_proxy_->GetEntityState(request->entity);
      if (gz_state) {
        ConvertState(*gz_state, response->state);
      } else {
        response->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Requested entity not found";
      }
    });

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "get_entity_state");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
