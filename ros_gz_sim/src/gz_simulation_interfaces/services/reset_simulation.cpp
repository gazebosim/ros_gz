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

#include "reset_simulation.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <future>
#include <memory>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/srv/reset_simulation.hpp"
namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using ResetSimulationSrv = simulation_interfaces::srv::ResetSimulation;
using RequestPtr = ResetSimulationSrv::Request::ConstSharedPtr;
using ResponsePtr = ResetSimulationSrv::Response::SharedPtr;

ResetSimulation::ResetSimulation(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  const auto control_service = this->gz_proxy_->PrefixTopic("control");
  if (!this->gz_proxy_->WaitForGzService(control_service)) {
    RCLCPP_ERROR_STREAM(
      this->ros_node_->get_logger(),
      "Gazebo service [" << control_service << "] is not available. "
                         << "The [ResetSimulation] interface will not function properly.");
  }
  this->services_handle_ = ros_node->create_service<ResetSimulationSrv>(
    "reset_simulation", [this, control_service](RequestPtr request, ResponsePtr response) {
      auto reset_detected_future = std::async(std::launch::async, [this]
      {
        return this->gz_proxy_->WaitForResetDetected();
      });

      using Result = simulation_interfaces::msg::Result;
      if (
        request->scope != ResetSimulationSrv::Request::SCOPE_DEFAULT &&
        request->scope != ResetSimulationSrv::Request::SCOPE_ALL)
      {
        response->result.result = Result::RESULT_FEATURE_UNSUPPORTED;
        response->result.error_message =
        "Only reset scopes SCOPE_DEFAULT and SCOPE_ALL are supported";
        return;
      }

      gz::msgs::WorldControl gz_request;
      gz_request.set_pause(this->gz_proxy_->Paused());
      gz_request.mutable_reset()->set_all(true);
      // TODO(azeey) Resetting only the time, state or spawned models is not supported yet in Gazebo

      bool result;
      gz::msgs::Boolean reply;
      bool executed = this->gz_proxy_->GzNode()->Request(
        control_service, gz_request, GazeboProxy::kGzServiceTimeoutMs, reply, result);
      if (!executed) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to reset simulation";
      } else if (result && reply.data()) {
        response->result.result = Result::RESULT_OK;
      } else {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Unknown error while trying to reset simulation";
      }

      // Since the "control" service is asynchronous, getting results from the service doesn't mean
      // the request has taken effect. Therefore, we wait here until the desired state is reached or
      // a timeout occurs.
      if (!reset_detected_future.get()) {
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
