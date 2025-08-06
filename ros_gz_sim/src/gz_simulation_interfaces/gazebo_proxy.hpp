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

#ifndef ROS_GZ_SIM__SIMULATION_INTERFACES_GAZEBO_STATE_HPP_
#define ROS_GZ_SIM__SIMULATION_INTERFACES_GAZEBO_STATE_HPP_

#include <gz/msgs/details/world_stats.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
class GazeboProxy
{
public:
  struct State
  {
    gz::math::Pose3d pose;
    gz::math::Vector3d linear_velocity;
    gz::math::Vector3d angular_velocity;
  };

  GazeboProxy(const std::string world_name, std::shared_ptr<rclcpp::Node> ros_node)
  : world_name_(world_name), ros_node_(ros_node), gz_node_(std::make_shared<gz::transport::Node>())
  {
    if (!this->InitializeGazeboConnection()) {
      throw std::runtime_error("Could not initialize Gazebo connection");
    }

    // TODO(azeey) Consider adding the UserCommands system if not already present.
    // TODO(azeey) Wait for critical services to be available (e.g. /world/*/create,
    // /world/*/control) Request the initial state of the world. This will block until Gazebo is
    // initialized
    gz::msgs::SerializedStepMap reply;
    bool result;
    if (!this->gz_node_->Request(this->PrefixTopic("state"), 30000, reply, result)) {
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

        std::cout << "Subscribe to " << this->PrefixTopic("state") << "\n";
        // Listen to the "state" topic to get periodic updates.
        if (!this->gz_node_->Subscribe(
              this->PrefixTopic("state"), &GazeboProxy::UpdateStateFromMsg, this)) {
          RCLCPP_ERROR(ros_node->get_logger(), "Subscribing to continues state updates failed");
        }
      }
    }
  }

  bool InitializeGazeboConnection()
  {
    gz::msgs::StringMsg_V worlds_msg;
    bool result;
    if (this->gz_node_->Request(
          "gazebo/worlds", GazeboProxy::kGzServiceTimeout, worlds_msg, result)) {
      if (result && !worlds_msg.data().empty()) {
        this->world_name_ = worlds_msg.data(0);
        return true;
      }
    }
    return false;
  }
  std::string PrefixTopic(const char * topic) { return "world/" + this->world_name_ + "/" + topic; }
  void UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg)
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    this->ecm_.SetState(msg.state());
    this->ecm_.ClearRemovedComponents();
    this->ecm_.ClearNewlyCreatedEntities();
    this->ecm_.ProcessRemoveEntityRequests();
    this->world_stats_ = msg.stats();

    // TODO(azeey) Consider using a condition variable to notify services that there is new data so
    // as to avoid using stale data
  }

  uint64_t Iterations()
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    return this->world_stats_.iterations();
  }
  bool Paused()
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    return this->world_stats_.paused();
  }

  void WithEcm(std::function<void(gz::sim::EntityComponentManager &)> f)
  {
    std::lock_guard<std::mutex> lk(this->stateSyncMutex_);
    f(this->ecm_);
  }

  std::shared_ptr<gz::transport::Node> GzNode() { return this->gz_node_; }

  static constexpr unsigned int kGzServiceTimeout{5000};

private:
  std::string world_name_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<gz::transport::Node> gz_node_;
  mutable std::mutex stateSyncMutex_;
  gz::sim::EntityComponentManager ecm_;
  gz::msgs::WorldStatistics world_stats_;
};
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif
