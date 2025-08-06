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

#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <memory>
#include <string>

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
  auto service_cb = [this](RequestPtr request, ResponsePtr response) {
    this->gz_proxy_->WithEcm([&](gz::sim::EntityComponentManager & ecm) {
      const auto entity = ecm.EntityByName(request->entity);

      if (entity) {
        ecm.SetComponentData<components::WorldPoseCmd>(*entity, ConvertPose(request->state.pose));
      } else {
        // TODO(azeey) Error
      }
      gz::msgs::WorldControlState control_msg;
      control_msg.mutable_state()->CopyFrom(ecm.State({*entity}, {components::WorldPoseCmd::typeId}));

      bool result;
      gz::msgs::Boolean reply;
      std::cout << "Sending: " << control_msg.DebugString() << std::endl;
      this->gz_proxy_->GzNode()->Request(this->gz_proxy_->PrefixTopic("control/state"), control_msg, 3000, reply, result);
      // TODO(azeey) Handle Error
      response->result.result = simulation_interfaces::msg::Result::RESULT_OK;
      // TODO(azeey) Wait for result?
    });
  };
  this->services_handle_ =
    ros_node->create_service<SetEntityStateSrv>("set_entity_state", service_cb);

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "set_entity_state");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
