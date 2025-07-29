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

#include "delete_entity.hpp"

#include <gz/msgs/boolean.pb.h>

#include "../gazebo_state.hpp"
#include "simulation_interfaces/srv/delete_entity.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using DeleteEntitySrv = simulation_interfaces::srv::DeleteEntity;
using RequestPtr = DeleteEntitySrv::Request::ConstSharedPtr;
using ResponsePtr = DeleteEntitySrv::Response::SharedPtr;

DeleteEntity::DeleteEntity(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboState> gz_state)
: HandlerBase(ros_node, gz_state)
{
  this->services_handle_ = ros_node->create_service<DeleteEntitySrv>(
    "delete_entity", [this](RequestPtr request, ResponsePtr response) {
      std::cout << "DeleteEntityCb called" << std::endl;
      gz::msgs::Entity gz_request;
      gz_request.set_name(request->entity);
      gz_request.set_type(gz::msgs::Entity::MODEL);
      gz::msgs::Boolean gz_reply;
      bool result;
      if (this->gz_state_->GzNode()->Request(
            this->gz_state_->PrefixTopic("remove"), gz_request, GazeboState::kGzServiceTimeout,
            gz_reply, result)) {
        if (result && gz_reply.data()) {
          response->result.result = simulation_interfaces::msg::Result::RESULT_OK;
          return;
        }
      }
      // TODO(azeey) Add specific error codes depending on what went wrong and add more thorough
      // error messages.
      response->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
      response->result.error_message = "Error while trying to remove entity";
    });

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "delete_entity");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
