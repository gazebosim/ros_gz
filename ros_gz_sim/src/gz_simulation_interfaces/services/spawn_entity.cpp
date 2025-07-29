// Coyright 2025 Open Source Robotics Foundation, Inc.
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

#include "spawn_entity.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include "../gazebo_state.hpp"
#include "simulation_interfaces/srv/spawn_entity.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using SpawnEntitySrv = simulation_interfaces::srv::SpawnEntity;
using RequestPtr = SpawnEntitySrv::Request::ConstSharedPtr;
using ResponsePtr = SpawnEntitySrv::Response::SharedPtr;

SpawnEntity::SpawnEntity(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboState> gz_state)
: HandlerBase(ros_node, gz_state)
{
  this->services_handle_ = ros_node->create_service<SpawnEntitySrv>(
    "spawn_entity", [this](RequestPtr request, ResponsePtr response) {
      using Result = simulation_interfaces::msg::Result;
      gz::msgs::EntityFactory gz_request;
      if (!request->name.empty()) {
        gz_request.set_name(request->name);
      }
      gz_request.set_allow_renaming(request->allow_renaming);

      if (!request->uri.empty()) {
        // TODO(azeey) The `sdf_filename` field requires absolute paths to the file.
        // Consider resolving the uri using the `/gazebo/resource_paths/resolve`
        gz_request.set_sdf_filename(request->uri);
      } else if (!request->resource_string.empty()) {
        gz_request.set_sdf(request->resource_string);
      } else {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message =
          "One of the fields [uri] or [resource_string] must be specified";
        return;
      }

      // TODO(azeey) Add support for entity_namespace
      // TODO(azeey) Reuse code in ros_gz_bridge/convert/geometry_msgs
      auto * pose = gz_request.mutable_pose();
      pose->mutable_position()->set_x(request->initial_pose.pose.position.x);
      pose->mutable_position()->set_y(request->initial_pose.pose.position.y);
      pose->mutable_position()->set_z(request->initial_pose.pose.position.z);

      pose->mutable_orientation()->set_x(request->initial_pose.pose.orientation.x);
      pose->mutable_orientation()->set_y(request->initial_pose.pose.orientation.y);
      pose->mutable_orientation()->set_z(request->initial_pose.pose.orientation.z);
      pose->mutable_orientation()->set_w(request->initial_pose.pose.orientation.w);

      bool result;
      gz::msgs::Boolean reply;
      bool executed = this->gz_state_->GzNode()->Request(
        this->gz_state_->PrefixTopic("create"), gz_request, 30000, reply, result);
      if (!executed) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to set simulation state";
      } else if (result && reply.data()) {
        response->result.result = Result::RESULT_OK;
        // TODO(azeey) Fetch the new name of the entity from our local ECM using `EachNew`.
        // We'd have to make sure that the ECM has been updated at least once after the `create`
        // request
        response->entity_name = request->name;
      } else {
        // TODO(azeey) SpawnEntity has additional error codes to allow surfacing more informative
        // error messages. However, the `create` service in UserCommands only returns a boolean.
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Unknown error while tryint to reset simulation";
      }
    });

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "spawn_entity");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
