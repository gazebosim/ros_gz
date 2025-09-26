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

#include "get_simulation_state.hpp"

#include <gz/msgs/boolean.pb.h>

#include <memory>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/srv/get_simulation_state.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetSimulationStateSrv = simulation_interfaces::srv::GetSimulationState;
using RequestPtr = GetSimulationStateSrv::Request::ConstSharedPtr;
using ResponsePtr = GetSimulationStateSrv::Response::SharedPtr;
using simulation_interfaces::msg::Result;

GetSimulationState::GetSimulationState(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  this->services_handle_ = ros_node->create_service<GetSimulationStateSrv>(
    "get_simulation_state", [this](RequestPtr, ResponsePtr response) {
      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }
      if (this->gz_proxy_->Paused()) {
        response->state.state = simulation_interfaces::msg::SimulationState::STATE_PAUSED;
        if (this->gz_proxy_->Iterations() == 0) {
          // The simulation is in its initial state after loading a world or being reset, which will
          // assign to the STATE_STOPPED state
          response->state.state = simulation_interfaces::msg::SimulationState::STATE_STOPPED;
        }
      } else {
        response->state.state = simulation_interfaces::msg::SimulationState::STATE_PLAYING;
      }

      response->result.result = Result::RESULT_OK;
    });

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
