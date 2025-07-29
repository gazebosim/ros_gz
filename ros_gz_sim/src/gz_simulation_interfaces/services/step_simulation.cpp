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

#include "step_simulation.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include "../gazebo_state.hpp"
#include "simulation_interfaces/srv/step_simulation.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using StepSimulationSrv = simulation_interfaces::srv::StepSimulation;
using RequestPtr = StepSimulationSrv::Request::ConstSharedPtr;
using ResponsePtr = StepSimulationSrv::Response::SharedPtr;

StepSimulation::StepSimulation(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboState> gz_state)
: HandlerBase(ros_node, gz_state)
{
  this->services_handle_ = ros_node->create_service<StepSimulationSrv>(
    "step_simulation", [this](RequestPtr request, ResponsePtr response) {
      using Result = simulation_interfaces::msg::Result;
      if (!this->gz_state_->Paused()) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Simulation has to be paused before stepping";
        return;
      }

      // The spec uses a uint64, but the service provided by Gazebo uses a uint32 so we bail out if
      // the requested number of steps cannot be represented properly.
      if (request->steps > std::numeric_limits<uint32_t>::max()) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message =
          "The requested number of steps exceeds the maximum supported value (max uint32)";
        return;
      }

      gz::msgs::WorldControl gz_request;
      gz_request.set_pause(true);
      gz_request.set_step(true);
      gz_request.set_multi_step(request->steps);
      bool result;
      gz::msgs::Boolean reply;
      bool executed = this->gz_state_->GzNode()->Request(
        this->gz_state_->PrefixTopic("control"), gz_request, 30000, reply, result);
      if (!executed) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to reset simulation";
      } else if (result && reply.data()) {
        response->result.result = Result::RESULT_OK;
      } else {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Unknown error while trying to reset simulation";
      }
    });

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "step_simulation");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
