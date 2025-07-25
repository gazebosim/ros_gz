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

#include "ros_gz_sim/simulation_interfaces.hpp"

#include <gz/msgs/details/boolean.pb.h>
#include <gz/msgs/details/entity.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <functional>
#include <gz/transport/Node.hh>
#include <iostream>

#include "simulation_interfaces/msg/result.hpp"
#include "simulation_interfaces/srv/delete_entity.hpp"

namespace ros_gz_sim
{

class SimulationInterfaces::Implementation
{
public:
  using DeleteEntity = simulation_interfaces::srv::DeleteEntity;
  void CreateServices(rclcpp::Node & node);

  bool InitializeGazeboParameters();

  void DeleteEntityCb(
    DeleteEntity::Request::ConstSharedPtr request, DeleteEntity::Response::SharedPtr response);

private:
  rclcpp::Service<DeleteEntity>::SharedPtr delete_entity_service_;
  gz::transport::Node node_;
  const unsigned int kTimeout_{5000};
  std::string world_name_;
};

void SimulationInterfaces::Implementation::CreateServices(rclcpp::Node & node)
{
  if (!this->InitializeGazeboParameters()) {
    // TODO(azeey) Log error
    return;
  }
  std::cout << "Creating services on " << node.get_name() << std::endl;
  this->delete_entity_service_ = node.create_service<DeleteEntity>(
    "delete_entity",
    std::bind(&Implementation::DeleteEntityCb, this, std::placeholders::_1, std::placeholders::_2));
}

bool SimulationInterfaces::Implementation::InitializeGazeboParameters()
{
  gz::msgs::StringMsg_V worlds_msg;
  bool result;
  if (this->node_.Request("gazebo/worlds", this->kTimeout_, worlds_msg, result)) {
    if (result && !worlds_msg.data().empty()) {
      this->world_name_ = worlds_msg.data(0);
      return true;
    }
  }
  return false;
}

void SimulationInterfaces::Implementation::DeleteEntityCb(
  DeleteEntity::Request::ConstSharedPtr request, DeleteEntity::Response::SharedPtr response)
{
  std::string topic = "world/" + this->world_name_ + "/remove";
  gz::msgs::Entity gz_request;
  gz_request.set_name(request->entity);
  gz_request.set_type(gz::msgs::Entity::MODEL);
  gz::msgs::Boolean gz_reply;
  bool result;
  if (node_.Request(topic, gz_request, this->kTimeout_, gz_reply, result)) {
    if (result && gz_reply.data()) {
      response->result.result = simulation_interfaces::msg::Result::RESULT_OK;
      return;
    }
  }
  // TODO(azeey) Add specific error codes depending on what went wrong and add more thorough error messages.
  response->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
  response->result.error_message = "Error while trying to remove entity";
}

SimulationInterfaces::SimulationInterfaces(rclcpp::Node & node)
: dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->CreateServices(node);
}
}  // namespace ros_gz_sim
