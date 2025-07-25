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
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <cstdint>
#include <functional>
#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

#include "simulation_interfaces/action//simulate_steps.hpp"
#include "simulation_interfaces/action/simulate_steps.hpp"
#include "simulation_interfaces/msg/result.hpp"
#include "simulation_interfaces/msg/simulation_state.hpp"
#include "simulation_interfaces/msg/simulator_features.hpp"
#include "simulation_interfaces/srv/delete_entity.hpp"
#include "simulation_interfaces/srv/get_entities.hpp"
#include "simulation_interfaces/srv/get_entities_states.hpp"
#include "simulation_interfaces/srv/get_entity_state.hpp"
#include "simulation_interfaces/srv/get_simulation_state.hpp"
#include "simulation_interfaces/srv/get_simulator_features.hpp"
#include "simulation_interfaces/srv/reset_simulation.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"
#include "simulation_interfaces/srv/spawn_entity.hpp"
#include "simulation_interfaces/srv/step_simulation.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{

class SimulationInterfaces::Implementation
{
public:
  // Services
  using DeleteEntity = simulation_interfaces::srv::DeleteEntity;
  using GetEntities = simulation_interfaces::srv::GetEntities;
  using GetEntityState = simulation_interfaces::srv::GetEntityState;
  using GetEntitiesStates = simulation_interfaces::srv::GetEntitiesStates;
  using GetSimulationState = simulation_interfaces::srv::GetSimulationState;
  using GetSimulatorFeatures = simulation_interfaces::srv::GetSimulatorFeatures;
  using ResetSimulation = simulation_interfaces::srv::ResetSimulation;
  using SetSimulationState = simulation_interfaces::srv::SetSimulationState;
  using SpawnEntity = simulation_interfaces::srv::SpawnEntity;
  using StepSimulation = simulation_interfaces::srv::StepSimulation;

  // Actions
  using SimulateSteps = simulation_interfaces::action::SimulateSteps;
  using GoalHandleSimulateSteps = rclcpp_action::ServerGoalHandle<SimulateSteps>;

  void Run(rclcpp::Node & node);
  std::string PrefixTopic(const char * topic);
  void UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg);
  void CreateServices(rclcpp::Node & node);
  template <typename Service, typename HandlerFunc>
  void AddService(rclcpp::Node & node, const char * service_name, HandlerFunc && callback);
  template <
    typename Action, typename GoalHandlerFunc, typename CancelHandlerFunc,
    typename AcceptedHandlerFunc>
  void AddAction(
    rclcpp::Node & node, const char * service_name, GoalHandlerFunc && goal_callback,
    CancelHandlerFunc && cancel_handler, AcceptedHandlerFunc && accepted_handler);

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
  void GetSimulatorFeaturesCb(
    GetSimulatorFeatures::Request::ConstSharedPtr request,
    GetSimulatorFeatures::Response::SharedPtr response);
  void ResetSimulationCb(
    ResetSimulation::Request::ConstSharedPtr request,
    ResetSimulation::Response::SharedPtr response);
  void SetSimulationStateCb(
    SetSimulationState::Request::ConstSharedPtr request,
    SetSimulationState::Response::SharedPtr response);
  void SpawnEntityCb(
    SpawnEntity::Request::ConstSharedPtr request, SpawnEntity::Response::SharedPtr response);
  void StepSimulationCb(
    StepSimulation::Request::ConstSharedPtr request, StepSimulation::Response::SharedPtr response);

  // Action handlers
  rclcpp_action::GoalResponse SimulateStepsGoalCb(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SimulateSteps::Goal> goal);
  rclcpp_action::CancelResponse SimulateStepsCancelCb(
    const std::shared_ptr<GoalHandleSimulateSteps> goal_handle);
  void SimulateStepsAcceptedCb(const std::shared_ptr<GoalHandleSimulateSteps> goal_handle);

  // Service helpers
  bool PopulateStateFromEcm(
    const gz::sim::Entity & entity, simulation_interfaces::msg::EntityState & state,
    simulation_interfaces::msg::Result & result);

private:
  // TODO(azeey) Consider storing the ROS node
  gz::transport::Node gz_node_;
  const unsigned int kTimeout_{5000};
  std::string world_name_;
  std::mutex stateSyncMutex_;
  gz::sim::EntityComponentManager ecm_;
  gz::msgs::WorldStatistics world_stats_;

  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_handles_;
  std::vector<std::shared_ptr<rclcpp_action::ServerBase>> action_handles_;
};

void SimulationInterfaces::Implementation::Run(rclcpp::Node & node)
{
  auto thread = std::thread([&] {
    if (!this->InitializeGazeboParameters()) {
      // TODO(azeey) Log error
      return;
    }

    // TODO(azeey) Consider adding the UserCommands system if not already present.
    // TODO(azeey) Wait for critical services to be available (e.g. /world/*/create,
    // /world/*/control) Request the initial state of the world. This will block until Gazebo is
    // initialized
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
  this->ecm_.ClearRemovedComponents();
  this->ecm_.ClearNewlyCreatedEntities();
  this->ecm_.ProcessRemoveEntityRequests();
  this->world_stats_ = msg.stats();

  // TODO(azeey) Consider using a condition variable to notify services that there is new data so as
  // to avoid using stale data
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
  this->AddService<GetSimulatorFeatures>(
    node, "get_simulator_features", &Implementation::GetSimulatorFeaturesCb);
  this->AddService<ResetSimulation>(node, "reset_simulation", &Implementation::ResetSimulationCb);
  this->AddService<SetSimulationState>(
    node, "set_simulation_state", &Implementation::SetSimulationStateCb);
  this->AddService<SpawnEntity>(node, "spawn_entity", &Implementation::SpawnEntityCb);
  this->AddService<StepSimulation>(node, "step_simulation", &Implementation::StepSimulationCb);

  this->AddAction<SimulateSteps>(
    node, "simulate_steps", &Implementation::SimulateStepsGoalCb,
    &Implementation::SimulateStepsCancelCb, &Implementation::SimulateStepsAcceptedCb);
}

template <typename Service, typename HandlerFunc>
void SimulationInterfaces::Implementation::AddService(
  rclcpp::Node & node, const char * service_name, HandlerFunc && callback)
{
  this->services_handles_.push_back(node.create_service<Service>(
    service_name, std::bind(callback, this, std::placeholders::_1, std::placeholders::_2)));

  RCLCPP_INFO_STREAM(node.get_logger(), "Created service " << service_name);
}

template <
  typename Action, typename GoalHandlerFunc, typename CancelHandlerFunc,
  typename AcceptedHandlerFunc>
void SimulationInterfaces::Implementation::AddAction(
  rclcpp::Node & node, const char * action_name, GoalHandlerFunc && goal_callback,
  CancelHandlerFunc && cancel_callback, AcceptedHandlerFunc && accepted_callback)
{
  this->action_handles_.push_back(
    rclcpp_action::create_server<Action>(
      &node, action_name,
      std::bind(goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(cancel_callback, this, std::placeholders::_1),
      std::bind(accepted_callback, this, std::placeholders::_1)));

  RCLCPP_INFO_STREAM(node.get_logger(), "Created action " << action_name);
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
  // TODO(azeey) Add specific error codes depending on what went wrong and add more thorough error
  // messages.
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
  // TODO (azeey) Since the name might not be unique across Gazebo entity types, ensure that the
  // matched entity is a model.
  auto entity = this->ecm_.EntityByName(request->entity);
  if (entity) {
    this->PopulateStateFromEcm(*entity, response->state, response->result);
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
      this->PopulateStateFromEcm(entity, state, response->result);

      // TODO(azeey) Implement error checking and setting error message
      return true;
    });
}

bool SimulationInterfaces::Implementation::PopulateStateFromEcm(
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
      // The simulation is in its initial state after loading a world or being reset, which will
      // assign to the STATE_STOPPED state
      response->state.state = simulation_interfaces::msg::SimulationState::STATE_STOPPED;
    }
  } else {
    response->state.state = simulation_interfaces::msg::SimulationState::STATE_PLAYING;
  }
}

void SimulationInterfaces::Implementation::GetSimulatorFeaturesCb(
  GetSimulatorFeatures::Request::ConstSharedPtr, GetSimulatorFeatures::Response::SharedPtr response)
{
  using SimulatorFeatures = simulation_interfaces::msg::SimulatorFeatures;
  // clang-format off
  response->features.features.assign({
    SimulatorFeatures::SPAWNING, SimulatorFeatures::DELETING,
    // SimulatorFeatures::ENTITY_TAGS, // TODO(azeey)
    // SimulatorFeatures::ENTITY_BOUNDS, // TODO(azeey)
    // SimulatorFeatures::ENTITY_BOUNDS_BOX, // TODO(azeey)
    // SimulatorFeatures::ENTITY_CATEGORIES, // TODO(azeey)
    SimulatorFeatures::SPAWNING_RESOURCE_STRING,
    SimulatorFeatures::ENTITY_STATE_GETTING,
    // SimulatorFeatures::ENTITY_STATE_SETTING, // TODO(azeey)
    // SimulatorFeatures::ENTITY_INFO_GETTING, // TODO(azeey)
    SimulatorFeatures::SIMULATION_RESET,
    // SimulatorFeatures::SIMULATION_RESET_TIME, // TODO(azeey)
    // SimulatorFeatures::SIMULATION_RESET_STATE, // TODO(azeey)
    // SimulatorFeatures::SIMULATION_RESET_SPAWNED, // TODO(azeey)
    SimulatorFeatures::SIMULATION_STATE_GETTING,
    // SimulatorFeatures::SIMULATION_STATE_SETTING, // TODO(azeey)
    SimulatorFeatures::SIMULATION_STATE_PAUSE,
    SimulatorFeatures::STEP_SIMULATION_SINGLE,
    SimulatorFeatures::STEP_SIMULATION_MULTIPLE,
    SimulatorFeatures::STEP_SIMULATION_ACTION,

    // clang-format on
  });

  response->features.spawn_formats.assign({"sdf", "urdf"});
  // TODO(azeey) Fill in custom_info
}

void SimulationInterfaces::Implementation::ResetSimulationCb(
  ResetSimulation::Request::ConstSharedPtr request, ResetSimulation::Response::SharedPtr response)
{
  using Result = simulation_interfaces::msg::Result;
  if (
    request->scope != ResetSimulation::Request::SCOPE_DEFAULT &&
    request->scope != ResetSimulation::Request::SCOPE_ALL) {
    response->result.result = Result::RESULT_FEATURE_UNSUPPORTED;
    response->result.error_message = "Only reset scopes SCOPE_DEFAULT and SCOPE_ALL are supported";
    return;
  }

  gz::msgs::WorldControl gz_request;
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    gz_request.set_pause(this->world_stats_.paused());
  }
  gz_request.mutable_reset()->set_all(true);
  // TODO(azeey) Reseting only the time, state or spawned models is not supported yet in Gazebo

  bool result;
  gz::msgs::Boolean reply;
  bool executed =
    this->gz_node_.Request(this->PrefixTopic("control"), gz_request, 30000, reply, result);
  if (!executed) {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Timed out while trying to reset simulation";
  } else if (result && reply.data()) {
    response->result.result = Result::RESULT_OK;
  } else {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Unknown error while tryint to reset simulation";
  }
}

void SimulationInterfaces::Implementation::SetSimulationStateCb(
  SetSimulationState::Request::ConstSharedPtr request,
  SetSimulationState::Response::SharedPtr response)
{
  using Result = simulation_interfaces::msg::Result;
  using SimulationState = simulation_interfaces::msg::SimulationState;

  gz::msgs::WorldControl gz_request;
  switch (request->state.state) {
    case SimulationState::STATE_STOPPED:
      gz_request.set_pause(true);
      gz_request.mutable_reset()->set_all(true);
      break;
    case SimulationState::STATE_PAUSED:
      gz_request.set_pause(true);
      break;
    case SimulationState::STATE_PLAYING:
      gz_request.set_pause(false);
      break;
    default:
      response->result.result = Result::RESULT_FEATURE_UNSUPPORTED;
      response->result.error_message =
        "Only the states [STATE_STOPPED, STATE_PAUSED, STATE_PLAYING] are supported";
      return;
  }

  bool result;
  gz::msgs::Boolean reply;
  bool executed =
    this->gz_node_.Request(this->PrefixTopic("control"), gz_request, 30000, reply, result);
  if (!executed) {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Timed out while trying to set simulation state";
  } else if (result && reply.data()) {
    response->result.result = Result::RESULT_OK;
  } else {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Unknown error while tryint to reset simulation";
  }
}

void SimulationInterfaces::Implementation::SpawnEntityCb(
  SpawnEntity::Request::ConstSharedPtr request, SpawnEntity::Response::SharedPtr response)
{
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
  bool executed =
    this->gz_node_.Request(this->PrefixTopic("create"), gz_request, 30000, reply, result);
  if (!executed) {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Timed out while trying to set simulation state";
  } else if (result && reply.data()) {
    response->result.result = Result::RESULT_OK;
    // TODO(azeey) Fetch the new name of the entity from our local ECM using `EachNew`.
    // We'd have to make sure that the ECM has been updated at least once after the `create` request
    response->entity_name = request->name;
  } else {
    // TODO(azeey) SpawnEntity has additional error codes to allow surfacing more informative error
    // messages. However, the `create` service in UserCommands only returns a boolean.
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Unknown error while tryint to reset simulation";
  }
}

void SimulationInterfaces::Implementation::StepSimulationCb(
  StepSimulation::Request::ConstSharedPtr request, StepSimulation::Response::SharedPtr response)
{
  using Result = simulation_interfaces::msg::Result;
  bool sim_paused;
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    sim_paused = this->world_stats_.paused();
  }
  if (!sim_paused) {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Simulation has to be paused before stepping";
    return;
  }

  // The spec uses a uint64, but the service provided by Gazebo uses a uint32 so we bail out if the
  // requested number of steps cannot be represented properly.
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
  bool executed =
    this->gz_node_.Request(this->PrefixTopic("control"), gz_request, 30000, reply, result);
  if (!executed) {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Timed out while trying to reset simulation";
  } else if (result && reply.data()) {
    response->result.result = Result::RESULT_OK;
  } else {
    response->result.result = Result::RESULT_OPERATION_FAILED;
    response->result.error_message = "Unknown error while trying to reset simulation";
  }
}

rclcpp_action::GoalResponse SimulationInterfaces::Implementation::SimulateStepsGoalCb(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const SimulateSteps::Goal>)
{
  // TODO(azeey) Add console message that we've received the goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimulationInterfaces::Implementation::SimulateStepsCancelCb(
  const std::shared_ptr<GoalHandleSimulateSteps>)
{
  // TODO(azeey) Add console message that we've received the cancellation
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SimulationInterfaces::Implementation::SimulateStepsAcceptedCb(
  const std::shared_ptr<GoalHandleSimulateSteps> goal_handle)
{
  // Note that this is not the same as SimulateSteps::Result, which is the typename associated with
  // the Result of the action, which in turn contains a simulation_interfaces::msg::Result
  using Result = simulation_interfaces::msg::Result;

  // Execute the task in a separate thread and return immediately.
  auto thread = std::thread([this, goal_handle] {
    gz::transport::Node action_gz_node;
    const auto goal = goal_handle->get_goal();
    auto action_result = std::make_shared<SimulateSteps::Result>();

    if (goal->steps > std::numeric_limits<uint32_t>::max()) {
      action_result->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
      action_result->result.error_message =
        "The requested number of steps exceeds the maximum supported value (max uint32)";
      goal_handle->abort(action_result);
      return;
    }

    bool sim_paused;
    {
      std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
      sim_paused = this->world_stats_.paused();
    }
    if (!sim_paused) {
      action_result->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
      action_result->result.error_message = "Simulation has to be paused before stepping";
      goal_handle->abort(action_result);
      return;
    }

    uint64_t num_iters_start = 0;
    {
      std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
      num_iters_start = this->world_stats_.iterations();
    }

    // TODO(azeey) Refactor this code since it's also used in the StepSimulation service.
    gz::msgs::WorldControl gz_request;
    gz_request.set_pause(true);
    gz_request.set_step(true);
    gz_request.set_multi_step(goal->steps);
    bool gz_result;
    gz::msgs::Boolean reply;
    bool executed =
      action_gz_node.Request(this->PrefixTopic("control"), gz_request, 30000, reply, gz_result);
    if (!executed) {
      action_result->result.result = Result::RESULT_OPERATION_FAILED;
      action_result->result.error_message = "Timed out while trying to reset simulation";
      goal_handle->abort(action_result);
    } else if (action_result && reply.data()) {
      // The request has succeeded, so now we listen to the stats and provide feedback.
      action_result->result.result = Result::RESULT_OK;
      auto feedback = std::make_shared<SimulateSteps::Feedback>();

      while (rclcpp::ok()) {
        std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
        auto iterations = this->world_stats_.iterations();
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
}

SimulationInterfaces::SimulationInterfaces(rclcpp::Node & node)
: dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->Run(node);
}
}  // namespace ros_gz_sim
