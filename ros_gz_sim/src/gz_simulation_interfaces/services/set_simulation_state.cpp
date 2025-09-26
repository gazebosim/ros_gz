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

#include "set_simulation_state.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <chrono>
#include <memory>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using SetSimulationStateSrv = simulation_interfaces::srv::SetSimulationState;
using RequestPtr = SetSimulationStateSrv::Request::ConstSharedPtr;
using ResponsePtr = SetSimulationStateSrv::Response::SharedPtr;

SetSimulationState::SetSimulationState(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  const auto control_service = this->gz_proxy_->PrefixTopic("control");
  if (!this->gz_proxy_->WaitForGzService(control_service)) {
    RCLCPP_ERROR_STREAM(
      this->ros_node_->get_logger(),
      "Gazebo service [" << control_service << "] is not available. "
                         << "The [SetSimulationState] interface will not function properly.");
  }
  this->services_handle_ = ros_node->create_service<SetSimulationStateSrv>(
    "set_simulation_state", [this, control_service](RequestPtr request, ResponsePtr response) {
      using Result = simulation_interfaces::msg::Result;
      using SimulationState = simulation_interfaces::msg::SimulationState;

      gz::msgs::WorldControl gz_request;
      switch (request->state.state) {
        case SimulationState::STATE_STOPPED:
          gz_request.set_pause(true);
          gz_request.mutable_reset()->set_all(true);
          break;
        case SimulationState::STATE_PAUSED:
          gz_request.set_pause(true);
          break;
        case SimulationState::STATE_PLAYING:
          gz_request.set_pause(false);
          break;
        default:
          response->result.result = Result::RESULT_FEATURE_UNSUPPORTED;
          response->result.error_message =
          "Only the states [STATE_STOPPED, STATE_PAUSED, STATE_PLAYING] are supported";
          return;
      }

      bool result;
      gz::msgs::Boolean reply;
      bool executed = this->gz_proxy_->GzNode()->Request(
        control_service, gz_request, GazeboProxy::kGzServiceTimeoutMs, reply, result);
      if (!executed) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to set simulation state";
      } else if (result && reply.data()) {
        response->result.result = Result::RESULT_OK;
      } else {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Unknown error while trying to reset simulation";
      }

      // Since the "control" service is asynchronous, getting results from the service doesn't mean
      // the request has taken effect. Therefore, we wait here until the desired state is reached or
      // a timeout occurs.
      bool state_reached{false};
      auto t_init = std::chrono::steady_clock::now();
      auto timeout = std::chrono::milliseconds(GazeboProxy::kGzStateUpdatedTimeoutMs);
      while (!state_reached && (std::chrono::steady_clock::now() - t_init) < timeout) {
        this->gz_proxy_->AssertUpdatedState(response->result);
        switch (request->state.state) {
          case SimulationState::STATE_STOPPED:
            if (this->gz_proxy_->Paused() && (this->gz_proxy_->Iterations() == 0)) {
              state_reached = true;
            }
            break;
          case SimulationState::STATE_PAUSED:
            if (this->gz_proxy_->Paused()) {
              state_reached = true;
            }
            break;
          case SimulationState::STATE_PLAYING:
            if (!this->gz_proxy_->Paused()) {
              state_reached = true;
            }
            break;
        }
      }
      if (!state_reached) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to reset simulation";
      }
    });

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
