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

#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <unordered_set>

#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interfaces/msg/result.hpp>

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
class GazeboProxy
{
public:
  GazeboProxy(const std::string world_name, std::shared_ptr<rclcpp::Node> ros_node);

  std::string PrefixTopic(const char * topic) const;

  bool WaitForService(
    const std::string & service,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(kGzServiceTimeoutMs));

  uint64_t Iterations() const;
  bool Paused() const;

  /// \brief Get a copy of the World statistics message.
  gz::msgs::WorldStatistics Stats() const;

  void WithEcm(std::function<void(gz::sim::EntityComponentManager &)> f);

  std::shared_ptr<gz::transport::Node> GzNode();

  bool StateInitialized() const;

  bool WaitForUpdatedState(
    const std::chrono::milliseconds & timeout =
    std::chrono::milliseconds(kGzStateUpdatedTimeoutMs));

  bool AssertUpdatedState(simulation_interfaces::msg::Result & result);

  static constexpr unsigned int kGzServiceTimeoutMs{5000};
  static constexpr unsigned int kGzStateUpdatedTimeoutMs{1000};

private:
  bool InitializeGazeboConnection();
  bool WaitForCriticalServices();
  void UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg);
  void HandleNewEntities();
  void InitializeCanonicalLinks(const std::unordered_set<gz::sim::Entity> & canonicalLinkEntities);

private:
  std::string world_name_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<gz::transport::Node> gz_node_;
  mutable std::mutex state_sync_mutex_;
  gz::sim::EntityComponentManager ecm_;
  gz::msgs::WorldStatistics world_stats_;
  bool state_intialized_{false};
  bool state_updated_{false};
  std::condition_variable state_cv_;
};
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif
