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

#include "ros_gz_sim/gz_simulation_interfaces.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

#include "actions/simulate_steps.hpp"
#include "gazebo_proxy.hpp"
#include "handler_base.hpp"
#include "services/delete_entity.hpp"
#include "services/get_entities.hpp"
#include "services/get_entities_states.hpp"
#include "services/get_entity_info.hpp"
#include "services/get_entity_state.hpp"
#include "services/get_simulation_state.hpp"
#include "services/get_simulator_features.hpp"
#include "services/reset_simulation.hpp"
#include "services/set_entity_state.hpp"
#include "services/set_simulation_state.hpp"
#include "services/spawn_entity.hpp"
#include "services/step_simulation.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{

namespace gz_simulation_interfaces
{
class GzSimulationInterfaces::Implementation
{
public:
  explicit Implementation(std::shared_ptr<rclcpp::Node> node);

  void Run();
  void UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg);
  void CreateInterfaces();

  template<typename Handler>
  void AddInterface();

  bool InitializeGazeboParameters();

private:
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::string world_name_;
  std::shared_ptr<GazeboProxy> gz_proxy_;
  std::vector<std::unique_ptr<HandlerBase>> sim_interface_handles_;
};

GzSimulationInterfaces::Implementation::Implementation(std::shared_ptr<rclcpp::Node> node)
: ros_node_(node)
{
}

void GzSimulationInterfaces::Implementation::Run()
{
  auto thread = std::thread([&] {
        try {
          this->gz_proxy_ = std::make_shared<GazeboProxy>(this->world_name_, this->ros_node_);
          this->CreateInterfaces();
        } catch (const std::exception & e) {
          RCLCPP_ERROR_STREAM(this->ros_node_->get_logger(), e.what());
        }
  });

  thread.detach();
}

void GzSimulationInterfaces::Implementation::CreateInterfaces()
{
  RCLCPP_INFO_STREAM(
    this->ros_node_->get_logger(), "Creating services on " << this->ros_node_->get_name());

  this->AddInterface<services::DeleteEntity>();
  this->AddInterface<services::GetEntities>();
  this->AddInterface<services::GetEntityInfo>();
  this->AddInterface<services::GetEntityState>();
  this->AddInterface<services::GetEntitiesStates>();
  this->AddInterface<services::GetSimulationState>();
  this->AddInterface<services::GetSimulatorFeatures>();
  this->AddInterface<services::ResetSimulation>();
  this->AddInterface<services::SetEntityState>();
  this->AddInterface<services::SetSimulationState>();
  this->AddInterface<services::SpawnEntity>();
  this->AddInterface<services::StepSimulation>();
  this->AddInterface<actions::SimulateSteps>();
}

template<typename Interface>
void GzSimulationInterfaces::Implementation::AddInterface()
{
  this->sim_interface_handles_.push_back(
    std::make_unique<Interface>(this->ros_node_, this->gz_proxy_));
}

GzSimulationInterfaces::GzSimulationInterfaces(std::shared_ptr<rclcpp::Node> node)
: dataPtr(gz::utils::MakeUniqueImpl<Implementation>(node))
{
  this->dataPtr->Run();
}
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
