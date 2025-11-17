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

#include <memory>

#include "../gazebo_proxy.hpp"
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
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  const auto remove_service = this->gz_proxy_->PrefixTopic("remove/blocking");
  if (!this->gz_proxy_->WaitForGzService(remove_service)) {
    RCLCPP_ERROR_STREAM(
      this->ros_node_->get_logger(),
      "Gazebo service ["
        << remove_service << "] is not available. "
        << "The [DeleteEntity] interface will not function properly. To fix this, make "
           "sure the [UserCommands] system is loaded in your Gazebo world");
  }
  this->services_handle_ = ros_node->create_service<DeleteEntitySrv>(
    "delete_entity", [this, remove_service](RequestPtr request, ResponsePtr response) {
      gz::msgs::Entity gz_request;
      gz_request.set_name(request->entity);
      gz_request.set_type(gz::msgs::Entity::MODEL);
      gz::msgs::Boolean gz_reply;
      bool result;
      if (this->gz_proxy_->GzNode()->Request(
            remove_service, gz_request, GazeboProxy::kGzServiceTimeoutMs, gz_reply, result))
      {
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

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
