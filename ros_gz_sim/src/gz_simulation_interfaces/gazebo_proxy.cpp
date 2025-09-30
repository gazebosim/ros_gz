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

#include "gazebo_proxy.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/details/world_stats.pb.h>
#include <gz/msgs/world_control_state.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/msgs/scene.pb.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <rclcpp/logging.hpp>

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace components = gz::sim::components;

GazeboProxy::GazeboProxy(const std::string world_name, std::shared_ptr<rclcpp::Node> ros_node)
: world_name_(world_name), ros_node_(ros_node), gz_node_(std::make_shared<gz::transport::Node>())
{
  if (!this->InitializeGazeboConnection()) {
    throw std::runtime_error("Could not initialize Gazebo connection");
  }

  if (!this->WaitForCriticalServices()) {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "Critical services from Gazebo are not available. ROS Simulation Interfaces will not "
      "function properly.");
  }
  // TODO(azeey) Consider adding the UserCommands system if not already present.
  // TODO(azeey) Wait for critical services to be available (e.g. /world/*/create,
  // /world/*/control) Request the initial state of the world. This will block until Gazebo is
  // initialized
  gz::msgs::SerializedStepMap reply;
  bool result;
  if (!this->gz_node_->Request(this->PrefixTopic("state"), kGzServiceTimeoutMs, reply, result)) {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "Simulation interface timed out while waiting for Gazebo to initialize");
    return;
  } else {
    if (!result) {
      RCLCPP_ERROR(
        ros_node->get_logger(),
        "Simulation interface encountered an error while synchronizing state with Gazebo");
      return;
    } else {
      this->UpdateStateFromMsg(reply);
      this->state_intialized_ = true;

      // Listen to the "state" topic to get periodic updates.
      this->SubscribeToGzTopic(this->PrefixTopic("state"), &GazeboProxy::UpdateStateFromMsg, this);
      // Listen to the "scene/info" to detect a reset
      // TODO(azeey): This is a hack. We currently don't have a nice way of determining when
      // simulation has been reset if it's currently paused. Checking if time has been rewound or
      // the number of iterations was reset back to zero doesn't work if the simulation was started
      // in the paused state and hasn't been played before it was reset. The SceneBroadacaster
      // publishes on the scene/info topic every time it's reset. So we'll use that as our signal
      // until we come up with a better way to communicate this from the server.
      std::function<void(const gz::msgs::Scene &)> resetHandler = [this](const auto &) {
          {
            std::lock_guard<std::mutex> lk(this->reset_detected_mutex_);
            this->reset_detected_ = true;
            this->reset_detected_cv_.notify_all();
          }
          // Use std::async since InitializeAllCanonicalLinks eventually makes a service call, which
          // we don't want to do from this callback thread.
          this->initialize_canonical_links_ =
            std::async(std::launch::async, [this] {this->InitializeAllCanonicalLinks();});
        };

      this->SubscribeToGzTopic(this->PrefixTopic("scene/info"), resetHandler);
    }

    std::function<void(const gz::msgs::WorldStatistics &)> updateStats =
      [this](const auto & stats)
      {
        std::lock_guard<std::mutex> lk(this->world_stats_sync_mutex_);
        this->world_stats_ = stats;
        this->world_stats_updated_ = true;
        this->world_stats_cv_.notify_all();
      };

    // Listen to the stats topic to get more frequently updates world statistics.
    this->SubscribeToGzTopic(this->PrefixTopic("stats"), updateStats);
  }

  // Before creating the services, we need to add the `[Angular/Linear]Velocity` components to all
  // the entities available. Currently, we're treating entities are models, but Gazebo doesn't
  // update velocity components of models. Therefore, we have to set the component on the canonical
  // link and compute the velocity of the model entity manually here.
  // TODO(azeey): Compute model velocities on the Gazebo server.
  this->InitializeAllCanonicalLinks();
}

std::string GazeboProxy::PrefixTopic(const char * topic) const
{
  return "world/" + this->world_name_ + "/" + topic;
}

bool GazeboProxy::WaitForGzService(
  const std::string & service, const std::chrono::milliseconds & timeout)
{
  std::vector<gz::transport::ServicePublisher> publishers;
  const auto start_time = std::chrono::system_clock::now();
  bool found_service = false;
  while (true) {
    this->GzNode()->ServiceInfo(service, publishers);
    if (!publishers.empty()) {
      found_service = true;
      break;
    }
    const auto now = std::chrono::system_clock::now();
    if (now > start_time + timeout) {break;}

    using namespace std::chrono_literals;  // NOLINT
    std::this_thread::sleep_for(500ms);
  }
  return found_service;
}
uint64_t GazeboProxy::Iterations() const
{
  std::lock_guard<std::mutex> lk(this->world_stats_sync_mutex_);
  return this->world_stats_.iterations();
}

bool GazeboProxy::Paused() const
{
  std::lock_guard<std::mutex> lk(this->world_stats_sync_mutex_);
  return this->world_stats_.paused();
}

gz::msgs::WorldStatistics GazeboProxy::Stats() const
{
  std::lock_guard<std::mutex> lk(this->world_stats_sync_mutex_);
  return this->world_stats_;
}

void GazeboProxy::WithEcm(std::function<void(gz::sim::EntityComponentManager &)> f)
{
  std::lock_guard<std::mutex> lk(this->state_sync_mutex_);
  f(this->ecm_);
}

std::shared_ptr<gz::transport::Node> GazeboProxy::GzNode() {return this->gz_node_;}

bool GazeboProxy::StateInitialized() const {return this->state_intialized_;}

bool GazeboProxy::WaitForUpdatedState(const std::chrono::milliseconds & timeout)
{
  // If the state has not been initialized, it will not be continuously updated, so return early.
  if (!state_intialized_) {
    return false;
  }
  if (this->initialize_canonical_links_.valid()) {
    // Wait if canonical links need to be initialized. This should only happen on reset.
    this->initialize_canonical_links_.wait_for(timeout);
  }
  // TODO(azeey): Technically we should subtract the amount of time the `wait_for` above used up
  // from the timeout when we use it for the second `wait_for` below.
  std::unique_lock lk(this->state_sync_mutex_);
  this->state_updated_ = false;
  return this->state_cv_.wait_for(lk, timeout, [this] {return this->state_updated_;});
}

bool GazeboProxy::AssertUpdatedState(simulation_interfaces::msg::Result & result)
{
  using simulation_interfaces::msg::Result;
  if (!this->StateInitialized()) {
    result.result = Result::RESULT_OPERATION_FAILED;
    result.error_message = "Required Gazebo system is missing";
    return false;
  }
  if (!this->WaitForUpdatedState()) {
    result.result = Result::RESULT_OPERATION_FAILED;
    result.error_message = "Timed out while waiting for updated Gazebo state";
    return false;
  }
  return true;
}

bool GazeboProxy::AssertUpdatedWorldStats(simulation_interfaces::msg::Result & result)
{
  using simulation_interfaces::msg::Result;
  std::unique_lock lk(this->world_stats_sync_mutex_);
  this->world_stats_updated_ = false;
  auto rc = this->world_stats_cv_.wait_for(
    lk, std::chrono::milliseconds(kGzStateUpdatedTimeoutMs),
    [this] {return this->world_stats_updated_;});
  if (!rc) {
    result.result = Result::RESULT_OPERATION_FAILED;
    result.error_message = "Timed out while waiting for updated Gazebo world stats";
    return false;
  }
  return true;
}

bool GazeboProxy::WaitForResetDetected()
{
  std::unique_lock lk(this->reset_detected_mutex_);
  this->reset_detected_ = false;
  if(!this->reset_detected_cv_.wait_for(
    lk, std::chrono::milliseconds(kGzServiceTimeoutMs), [this] {return this->reset_detected_;}))
  {
    return false;
  }
  return this->reset_detected_;
}

bool GazeboProxy::InitializeGazeboConnection()
{
  gz::msgs::StringMsg_V worlds_msg;
  bool result;
  if (this->gz_node_->Request(
        "gazebo/worlds", GazeboProxy::kGzServiceTimeoutMs, worlds_msg, result))
  {
    if (result && !worlds_msg.data().empty()) {
      this->world_name_ = worlds_msg.data(0);
      return true;
    }
  }
  return false;
}

bool GazeboProxy::WaitForCriticalServices()
{
  bool have_all_services = true;
  // Check that services from SceneBroadacaster are available
  {
    const auto state_service = this->PrefixTopic("state");
    if (!this->WaitForGzService(state_service)) {
      RCLCPP_ERROR_STREAM(
        this->ros_node_->get_logger(),
        "Required Gazebo service ["
          << state_service << "] is not available. "
          << "Make sure the [SceneBroadacaster] system is loaded in your Gazebo world");
      have_all_services = false;
    }
  }

  {
    const auto control_service = this->PrefixTopic("control/state");
    if (!this->WaitForGzService(control_service)) {
      RCLCPP_ERROR_STREAM(
        this->ros_node_->get_logger(),
        "Gazebo service [" << control_service << "] is not available.");

      have_all_services = false;
    }
  }

  return have_all_services;
}

void GazeboProxy::UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg)
{
  {
    std::lock_guard<std::mutex> lk(this->state_sync_mutex_);
    this->ecm_.SetState(msg.state());

    this->HandleNewEntities();
    this->ecm_.ClearRemovedComponents();
    this->ecm_.ClearNewlyCreatedEntities();
    this->ecm_.ProcessRemoveEntityRequests();
    this->state_updated_ = true;
  }
  this->state_cv_.notify_all();
}
void GazeboProxy::HandleNewEntities()
{
  std::unordered_set<gz::sim::Entity> canonicalLinkEntities;
  this->ecm_.EachNew<components::CanonicalLink>(
    [&canonicalLinkEntities](const gz::sim::Entity & entity, const components::CanonicalLink *) {
      canonicalLinkEntities.insert(entity);
      return true;
    });

  if (canonicalLinkEntities.empty()) {
    return;
  }

  this->InitializeCanonicalLinks(canonicalLinkEntities);
}

void GazeboProxy::InitializeAllCanonicalLinks()
{
  RCLCPP_INFO(this->ros_node_->get_logger(), "InitializeCanonicalLinks");
  std::vector<gz::sim::Entity> c_links;
  {
    std::lock_guard<std::mutex> lk(this->state_sync_mutex_);
    c_links = this->ecm_.EntitiesByComponents(components::CanonicalLink());
  }
  this->InitializeCanonicalLinks({c_links.begin(), c_links.end()});
}

void GazeboProxy::InitializeCanonicalLinks(
  const std::unordered_set<gz::sim::Entity> & canonicalLinkEntities)
{
  // TODO(azeey) Computing velocities at every timestep might have a performance impact
  for (const auto & link : canonicalLinkEntities) {
    this->ecm_.CreateComponent(link, components::WorldPose());
    this->ecm_.CreateComponent(link, components::WorldLinearVelocity());
    this->ecm_.CreateComponent(link, components::WorldAngularVelocity());
  }

  gz::msgs::WorldControlState control_msg;
  control_msg.mutable_state()->CopyFrom(this->ecm_.State(
    canonicalLinkEntities, {components::WorldPose::typeId, components::WorldLinearVelocity::typeId,
        components::WorldAngularVelocity::typeId}));

  bool result{false};
  gz::msgs::Boolean controlReply;
  this->gz_node_->Request(
    this->PrefixTopic("control/state"), control_msg, GazeboProxy::kGzServiceTimeoutMs, controlReply,
    result);
  if (!result || !controlReply.data()) {
    RCLCPP_ERROR_THROTTLE(
      this->ros_node_->get_logger(), *this->ros_node_->get_clock(), 1000,
      "Simulation interface encountered an error while handling newly created entities");
    return;
  }
}
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
