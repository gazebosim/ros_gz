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

#include "simulate_steps.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/action/simulate_steps.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace actions
{
using SimulateStepsAction = simulation_interfaces::action::SimulateSteps;
using GoalHandleSimulateSteps = rclcpp_action::ServerGoalHandle<SimulateStepsAction>;

SimulateSteps::SimulateSteps(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  auto goal_callback =
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const SimulateStepsAction ::Goal>) {
      // TODO(azeey) Add console message that we've received the goal
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  auto cancel_cb = [](const std::shared_ptr<GoalHandleSimulateSteps>) {
    // TODO(azeey) Add console message that we've received the cancellation
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto accept_cb = [this](const std::shared_ptr<GoalHandleSimulateSteps> goal_handle) {
    // Note that this is not the same as SimulateStepsAction::Result, which is the typename
    // associated with the Result of the action, which in turn contains a
    // simulation_interfaces::msg::Result
    using Result = simulation_interfaces::msg::Result;

    // Execute the task in a separate thread and return immediately.
    auto thread = std::thread([this, goal_handle] {
      const auto goal = goal_handle->get_goal();
      auto action_result = std::make_shared<SimulateStepsAction::Result>();

      if (goal->steps > std::numeric_limits<uint32_t>::max()) {
        action_result->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
        action_result->result.error_message =
          "The requested number of steps exceeds the maximum supported value (max uint32)";
        goal_handle->abort(action_result);
        return;
      }

      if (!this->gz_proxy_->Paused()) {
        action_result->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
        action_result->result.error_message = "Simulation has to be paused before stepping";
        goal_handle->abort(action_result);
        return;
      }

      uint64_t num_iters_start = this->gz_proxy_->Iterations();

      // TODO(azeey) Refactor this code since it's also used in the StepSimulation service.
      gz::msgs::WorldControl gz_request;
      gz_request.set_pause(true);
      gz_request.set_step(true);
      gz_request.set_multi_step(goal->steps);
      bool gz_result;
      gz::msgs::Boolean reply;
      bool executed = this->gz_proxy_->GzNode()->Request(
        this->gz_proxy_->PrefixTopic("control"), gz_request, 30000, reply, gz_result);
      if (!executed) {
        action_result->result.result = Result::RESULT_OPERATION_FAILED;
        action_result->result.error_message = "Timed out while trying to reset simulation";
        goal_handle->abort(action_result);
      } else if (action_result && reply.data()) {
        // The request has succeeded, so now we listen to the stats and provide feedback.
        action_result->result.result = Result::RESULT_OK;
        auto feedback = std::make_shared<SimulateStepsAction::Feedback>();

        while (rclcpp::ok()) {
          auto iterations = this->gz_proxy_->Iterations();
          feedback->completed_steps = iterations - num_iters_start;
          feedback->remaining_steps = goal->steps - feedback->completed_steps;
          goal_handle->publish_feedback(feedback);
          // TODO(azeey) There is a bug in Gazebo where the stepping field is set to true only once
          // immediately after the request to step instead of being true for the whole duration of
          // steps. So we can't use this right now to determine if we need to break out early
          // if (!this->world_stats_.stepping()) {
          //   break;
          // }
          if (feedback->remaining_steps == 0) {
            break;
          }

          if (goal_handle->is_canceling()) {
            goal_handle->canceled(action_result);
            return;
          }
        }

        if (rclcpp::ok()) {
          if (feedback->remaining_steps == 0) {
            goal_handle->succeed(action_result);
          } else {
            action_result->result.result = Result::RESULT_OPERATION_FAILED;
            action_result->result.error_message = "SimulateSteps was interrupted";
            goal_handle->abort(action_result);
          }
        }
      } else {
        action_result->result.result = Result::RESULT_OPERATION_FAILED;
        action_result->result.error_message = "Unknown error while trying to reset simulation";
        goal_handle->abort(action_result);
      }
    });
    thread.detach();
  };

  this->action_handles_ = rclcpp_action::create_server<SimulateStepsAction>(
    ros_node, "simulate_steps", goal_callback, cancel_cb, accept_cb);

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created action " << "simulate_steps");
}
}  // namespace actions
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
