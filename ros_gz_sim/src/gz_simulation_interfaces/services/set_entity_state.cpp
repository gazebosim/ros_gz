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

#include "set_entity_state.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/world_control_state.pb.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>

#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/PoseCmd.hh>

#include "../gazebo_proxy.hpp"
#include "../utils.hpp"
#include "simulation_interfaces/srv/set_entity_state.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using SetEntityStateSrv = simulation_interfaces::srv::SetEntityState;
using RequestPtr = SetEntityStateSrv::Request::ConstSharedPtr;
using ResponsePtr = SetEntityStateSrv::Response::SharedPtr;

using simulation_interfaces::msg::Result;
namespace components = gz::sim::components;

SetEntityState::SetEntityState(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  const auto control_state_service = this->gz_proxy_->PrefixTopic("control/state");
  if (!this->gz_proxy_->WaitForGzService(control_state_service)) {
    RCLCPP_ERROR_STREAM(
      this->ros_node_->get_logger(),
      "Gazebo service [" << control_state_service << "] is not available. "
                         << "The [SetEntityState] interface will not function properly.");
  }
  auto service_cb = [this, control_state_service](RequestPtr request, ResponsePtr response) {
      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }

      if (!request->state.header.frame_id.empty() &&
        request->state.header.frame_id != "world")
      {
        response->result.result = Result::RESULT_FEATURE_UNSUPPORTED;
        response->result.error_message = "Only the world reference frame is supported";
        return;
      }

      const auto & orientation = request->state.pose.orientation;
      const auto orientation_norm_squared =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
      if (
        request->set_pose &&
        (!std::isfinite(orientation_norm_squared) ||
        orientation_norm_squared <= std::numeric_limits<double>::epsilon()))
      {
        response->result.result = SetEntityStateSrv::Response::INVALID_POSE;
        response->result.error_message = "Pose orientation must be a valid quaternion";
        return;
      }

      gz::msgs::WorldControlState control_msg;
      bool entity_found = false;
      bool invalid_static_state = false;

      this->gz_proxy_->WithEcm(
        [&](gz::sim::EntityComponentManager & ecm) {
          const auto entity = ecm.EntityByName(request->entity);
          if (!entity) {
            return;
          }
          entity_found = true;

          gz::sim::Model model(*entity);
          const auto nonzero = [](const auto & vector) {
            return vector.x != 0.0 || vector.y != 0.0 || vector.z != 0.0;
          };
          if (
            model.Static(ecm) &&
            ((request->set_twist &&
            (nonzero(request->state.twist.linear) || nonzero(request->state.twist.angular))) ||
            (request->set_acceleration &&
            (nonzero(request->state.acceleration.linear) ||
            nonzero(request->state.acceleration.angular)))))
          {
            invalid_static_state = true;
            return;
          }

          std::unordered_set<gz::sim::ComponentTypeId> component_types;
          if (request->set_pose) {
            model.SetWorldPoseCmd(ecm, ConvertPose(request->state.pose));
            component_types.insert(components::WorldPoseCmd::typeId);
          }

          if (request->set_twist && !model.Static(ecm)) {
            // Velocity components are expected to be in the body frame, so transform them from
            // the world frame using the requested pose when it is being set at the same time.
            const auto entity_world_pose = request->set_pose ?
            ConvertPose(request->state.pose) : gz::sim::worldPose(*entity, ecm);
            const auto linear_vel_cmd_body = entity_world_pose.Rot().RotateVectorReverse(
              ConvertVector3(request->state.twist.linear));
            const auto angular_vel_cmd_body = entity_world_pose.Rot().RotateVectorReverse(
              ConvertVector3(request->state.twist.angular));

            ecm.SetComponentData<components::LinearVelocityCmd>(*entity, linear_vel_cmd_body);
            ecm.SetComponentData<components::AngularVelocityCmd>(*entity, angular_vel_cmd_body);
            component_types.insert(components::LinearVelocityCmd::typeId);
            component_types.insert(components::AngularVelocityCmd::typeId);
          }

          if (component_types.empty()) {
            return;
          }
          control_msg.mutable_state()->CopyFrom(ecm.State(
            {*entity}, component_types));
        });

      if (!entity_found) {
        response->result.result = Result::RESULT_NOT_FOUND;
        response->result.error_message = "Requested entity was not found";
        return;
      }
      if (invalid_static_state) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message =
          "Cannot set non-zero twist or acceleration on static entity";
        return;
      }
      if (!request->set_pose && !request->set_twist) {
        response->result.result = Result::RESULT_OK;
        return;
      }

      control_msg.mutable_world_control()->set_pause(this->gz_proxy_->Paused());
      bool result;
      gz::msgs::Boolean reply;
      const bool executed = this->gz_proxy_->GzNode()->Request(
        control_state_service, control_msg, GazeboProxy::kGzServiceTimeoutMs, reply, result);
      if (!executed) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to set entity state";
      } else if (result && reply.data()) {
        response->result.result = Result::RESULT_OK;
      } else {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Gazebo failed to set entity state";
      }
    };
  this->services_handle_ =
    ros_node->create_service<SetEntityStateSrv>("set_entity_state", service_cb);

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
