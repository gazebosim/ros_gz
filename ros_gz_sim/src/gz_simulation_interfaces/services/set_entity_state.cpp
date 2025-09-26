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

#include <memory>
#include <string>

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

      gz::msgs::WorldControlState control_msg;

      this->gz_proxy_->WithEcm(
        [&](gz::sim::EntityComponentManager & ecm) {
          const auto entity = ecm.EntityByName(request->entity);

          // TODO(azeey) Handle frame semantics. For now we assume all commands are in the world
          // frame.

          // Note that since there is no way to tell if a field has been set by the user, there's no
          // setting just the pose or just the twist. They will both be set according to what's in
          // the message. If not set by the user, the default values will be used.
          if (entity) {
            gz::sim::Model model(*entity);
            model.SetWorldPoseCmd(ecm, ConvertPose(request->state.pose));
            if (!model.Static(ecm)) {
              // Velocity components are expected to be in the body frame, so we'll need to
              // transform them.
              // TODO(azeey) Clarify whether the velocities are set in the new pose of the entity
              auto entityWorldPose = gz::sim::worldPose(*entity, ecm);
              auto linearVelCmdBody =
              entityWorldPose.Rot().RotateVectorReverse(ConvertVector3(
                  request->state.twist.linear));
              auto angularVelCmdBody =
              entityWorldPose.Rot().RotateVectorReverse(ConvertVector3(
                  request->state.twist.angular));

              ecm.SetComponentData<components::LinearVelocityCmd>(*entity, linearVelCmdBody);
              ecm.SetComponentData<components::AngularVelocityCmd>(*entity, angularVelCmdBody);
            }
          } else {
            // TODO(azeey) Error
          }
          control_msg.mutable_state()->CopyFrom(ecm.State(
            {*entity}, {components::WorldPoseCmd::typeId, components::LinearVelocityCmd::typeId,
              components::AngularVelocityCmd::typeId}));
        });

      control_msg.mutable_world_control()->set_pause(this->gz_proxy_->Paused());
      bool result;
      gz::msgs::Boolean reply;
      this->gz_proxy_->GzNode()->Request(control_state_service, control_msg,
          GazeboProxy::kGzServiceTimeoutMs, reply, result);
      // TODO(azeey) Handle Error
      response->result.result = simulation_interfaces::msg::Result::RESULT_OK;
      // TODO(azeey) Wait for result?
    };
  this->services_handle_ =
    ros_node->create_service<SetEntityStateSrv>("set_entity_state", service_cb);

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
