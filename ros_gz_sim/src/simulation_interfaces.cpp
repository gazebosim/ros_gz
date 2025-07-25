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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <functional>
#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <memory>
#include <mutex>

#include "simulation_interfaces/msg/result.hpp"
#include "simulation_interfaces/msg/simulation_state.hpp"
#include "simulation_interfaces/srv/delete_entity.hpp"
#include "simulation_interfaces/srv/get_entities.hpp"
#include "simulation_interfaces/srv/get_entities_states.hpp"
#include "simulation_interfaces/srv/get_entity_state.hpp"
#include "simulation_interfaces/srv/get_simulation_state.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{

class SimulationInterfaces::Implementation
{
public:
  using DeleteEntity = simulation_interfaces::srv::DeleteEntity;
  using GetEntities = simulation_interfaces::srv::GetEntities;
  using GetEntityState = simulation_interfaces::srv::GetEntityState;
  using GetEntitiesStates = simulation_interfaces::srv::GetEntitiesStates;
  using GetSimulationState = simulation_interfaces::srv::GetSimulationState;

  void Run(rclcpp::Node & node);
  std::string PrefixTopic(const char * topic);
  void UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg);
  void CreateServices(rclcpp::Node & node);
  template <typename Service, typename HandlerFunc>
  void AddService(rclcpp::Node & node, const char * service_name, HandlerFunc && callback);
  bool InitializeGazeboParameters();

  // Service handlers
  void DeleteEntityCb(
    DeleteEntity::Request::ConstSharedPtr request, DeleteEntity::Response::SharedPtr response);
  void GetEntitiesCb(
    GetEntities::Request::ConstSharedPtr request, GetEntities::Response::SharedPtr response);
  void GetEntityStateCb(
    GetEntityState::Request::ConstSharedPtr request, GetEntityState::Response::SharedPtr response);
  void GetEntitiesStatesCb(
    GetEntitiesStates::Request::ConstSharedPtr request,
    GetEntitiesStates::Response::SharedPtr response);
  void GetSimulationStateCb(
    GetSimulationState::Request::ConstSharedPtr request,
    GetSimulationState::Response::SharedPtr response);

  // Service helpers
  bool SetStateOfEntity(
    const gz::sim::Entity & entity, simulation_interfaces::msg::EntityState & state,
    simulation_interfaces::msg::Result & result);

private:
  gz::transport::Node gz_node_;
  const unsigned int kTimeout_{5000};
  std::string world_name_;
  std::mutex stateSyncMutex_;
  gz::sim::EntityComponentManager ecm_;
  gz::msgs::WorldStatistics world_stats_;

  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_handles_;
};

void SimulationInterfaces::Implementation::Run(rclcpp::Node & node)
{
  auto thread = std::thread([&] {
    if (!this->InitializeGazeboParameters()) {
      // TODO(azeey) Log error
      return;
    }
    // Request the initial state of the world. This will block until Gazebo is initialized
    gz::msgs::SerializedStepMap reply;
    bool result;
    if (!this->gz_node_.Request(this->PrefixTopic("state"), 30000, reply, result)) {
      RCLCPP_ERROR(
        node.get_logger(), "Simulation interface timed out while waiting for Gazebo to initialize");
      return;
    } else {
      if (!result) {
        RCLCPP_ERROR(
          node.get_logger(),
          "Simulation interface encountered an error while synchronizing state with Gazebo");
        return;
      } else {
        this->UpdateStateFromMsg(reply);

        std::cout << "Subscribe to " << this->PrefixTopic("state") << "\n";
        // Listen to the "state" topic to get periodic updates.
        if (!this->gz_node_.Subscribe(
              this->PrefixTopic("state"), &SimulationInterfaces::Implementation::UpdateStateFromMsg,
              this)) {
          RCLCPP_ERROR(node.get_logger(), "Subscribing to continues state updates failed");
        }

        this->CreateServices(node);
      }
    }
  });

  thread.detach();
}

std::string SimulationInterfaces::Implementation::PrefixTopic(const char * topic)
{
  return "world/" + this->world_name_ + "/" + topic;
}

void SimulationInterfaces::Implementation::UpdateStateFromMsg(
  const gz::msgs::SerializedStepMap & msg)
{
  // TODO(azeey) `msg` also contains stats
  std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
  this->ecm_.SetState(msg.state());
  this->world_stats_ = msg.stats();

  // TODO(azeey) Consider using a condition variable to notify services that there is new data so as to avoid using stale data
}

void SimulationInterfaces::Implementation::CreateServices(rclcpp::Node & node)
{
  RCLCPP_INFO_STREAM(node.get_logger(), "Creating services on " << node.get_name());
  this->AddService<DeleteEntity>(node, "delete_entity", &Implementation::DeleteEntityCb);
  this->AddService<GetEntities>(node, "get_entities", &Implementation::GetEntitiesCb);
  this->AddService<GetEntityState>(node, "get_entity_state", &Implementation::GetEntityStateCb);
  this->AddService<GetEntitiesStates>(
    node, "get_entities_states", &Implementation::GetEntitiesStatesCb);
  this->AddService<GetSimulationState>(
    node, "get_simulation_state", &Implementation::GetSimulationStateCb);
}

template <typename Service, typename HandlerFunc>
void SimulationInterfaces::Implementation::AddService(
  rclcpp::Node & node, const char * service_name, HandlerFunc && callback)
{
  this->services_handles_.push_back(node.create_service<Service>(
    service_name, std::bind(callback, this, std::placeholders::_1, std::placeholders::_2)));

  RCLCPP_INFO_STREAM(node.get_logger(), "Created service " << service_name);
}

bool SimulationInterfaces::Implementation::InitializeGazeboParameters()
{
  gz::msgs::StringMsg_V worlds_msg;
  bool result;
  if (this->gz_node_.Request("gazebo/worlds", this->kTimeout_, worlds_msg, result)) {
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
  std::cout << "DeleteEntityCb called" << std::endl;
  gz::msgs::Entity gz_request;
  gz_request.set_name(request->entity);
  gz_request.set_type(gz::msgs::Entity::MODEL);
  gz::msgs::Boolean gz_reply;
  bool result;
  if (gz_node_.Request(
        this->PrefixTopic("remove"), gz_request, this->kTimeout_, gz_reply, result)) {
    if (result && gz_reply.data()) {
      response->result.result = simulation_interfaces::msg::Result::RESULT_OK;
      return;
    }
  }
  // TODO(azeey) Add specific error codes depending on what went wrong and add more thorough error messages.
  response->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
  response->result.error_message = "Error while trying to remove entity";
}

void SimulationInterfaces::Implementation::GetEntitiesCb(
  GetEntities::Request::ConstSharedPtr, GetEntities::Response::SharedPtr response)
{
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    this->ecm_.Each<components::Name, components::Model>(
      [&](const gz::sim::Entity &, const components::Name * name, const components::Model *) {
        response->entities.push_back(name->Data());
        return true;
      });

    // TODO(azeey) Ensure that entities listed are top level models
    // TODO(azeey) Implement filtering by name
    // TODO(azeey) Implement filtering by category
    // TODO(azeey) Implement filtering by bounds
    // TODO(azeey) Implement error checking and setting error message
  }
}

void SimulationInterfaces::Implementation::GetEntityStateCb(
  GetEntityState::Request::ConstSharedPtr request, GetEntityState::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
  // TODO (azeey) Since the name might not be unique across Gazebo entity types, ensure that the matched entity is a model.
  auto entity = this->ecm_.EntityByName(request->entity);
  if (entity) {
    this->SetStateOfEntity(*entity, response->state, response->result);
  } else {
    response->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Requested entity not found";
  }
}

void SimulationInterfaces::Implementation::GetEntitiesStatesCb(
  GetEntitiesStates::Request::ConstSharedPtr, GetEntitiesStates::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
  this->ecm_.Each<components::Name, components::Model>(
    [&](const gz::sim::Entity & entity, const components::Name * name, const components::Model *) {
      response->entities.push_back(name->Data());
      auto & state = response->states.emplace_back();
      this->SetStateOfEntity(entity, state, response->result);

      // TODO(azeey) Implement error checking and setting error message
      return true;
    });
}

bool SimulationInterfaces::Implementation::SetStateOfEntity(
  const gz::sim::Entity & entity, simulation_interfaces::msg::EntityState & state,
  simulation_interfaces::msg::Result &)
{
  auto pose = gz::sim::worldPose(entity, this->ecm_);

  // TODO(azeey) Fill in header
  state.pose.position.x = pose.X();
  state.pose.position.y = pose.Y();
  state.pose.position.z = pose.Z();

  state.pose.orientation.x = pose.Rot().X();
  state.pose.orientation.y = pose.Rot().Y();
  state.pose.orientation.z = pose.Rot().Z();
  state.pose.orientation.w = pose.Rot().W();

  // TODO(azeey) Add support for twists and accelerations
  // TODO(azeey) Implement error checking and setting error message
  return true;
}

void SimulationInterfaces::Implementation::GetSimulationStateCb(
  GetSimulationState::Request::ConstSharedPtr, GetSimulationState::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
  if (this->world_stats_.paused()) {
    response->state.state = simulation_interfaces::msg::SimulationState::STATE_PAUSED;
    if (this->world_stats_.iterations() == 0) {
      // The simulation is in its initial state after loading a world or being reset, which will assign to the STATE_STOPPED state
      response->state.state = simulation_interfaces::msg::SimulationState::STATE_STOPPED;
    }
  } else {
    response->state.state = simulation_interfaces::msg::SimulationState::STATE_PLAYING;
  }
}

SimulationInterfaces::SimulationInterfaces(rclcpp::Node & node)
: dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->Run(node);
}
}  // namespace ros_gz_sim
