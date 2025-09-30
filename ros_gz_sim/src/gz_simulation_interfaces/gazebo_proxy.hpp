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

#ifndef GZ_SIMULATION_INTERFACES__GAZEBO_PROXY_HPP_
#define GZ_SIMULATION_INTERFACES__GAZEBO_PROXY_HPP_

#include <gz/msgs/world_stats.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

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
/// \class GazeboProxy
/// \brief Class that serves as a proxy to the Gazebo server by providing local caches of essential
/// data or state. It provides:
///   - A local EntityComponentManager (ECM) that is synchronized with the server
///   - A WorldStatistics object that is updated regularly
/// The class provides access to these objects in a thread-safe manner.
/// Additionally, the class has convenience functions that are used by most of the Simulation
/// Interface services/actions.
class GazeboProxy
{
public:
  /// \brief Constructor.
  /// \param[in] world_name The name of the Gazebo world that is being simulated.
  /// \param[in] ros_node ROS node.
  GazeboProxy(const std::string world_name, std::shared_ptr<rclcpp::Node> ros_node);

  /// \brief Prefixes a given topic with the world name.
  /// \param[in] topic The topic to be prefixed.
  /// \return The prefixed topic. e.g. Given "create" as the topic and the world name is "shapes",
  /// this will return "/world/shapes/create".
  std::string PrefixTopic(const char * topic) const;

  /// \brief Wait for a Gazebo service.
  /// \param[in] service Full name of the service.
  /// \param[in] timeout Amount of time to wait before giving up.
  /// \return True if the service was found before a timeout occurred.
  bool WaitForGzService(
    const std::string & service,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(kGzServiceTimeoutMs));

  /// \brief Get the number of iterations so far.
  /// \return Number of iterations executed by Gazebo.
  /// \WARNING Calling this function within a callback of the WithEcm will cause a deadlock.
  uint64_t Iterations() const;

  /// \brief Get whether simulation is in a paused state.
  /// \return Paused state.
  /// \WARNING Calling this function within a callback of the WithEcm will cause a deadlock.
  bool Paused() const;

  /// \brief Get a copy of the World statistics message.
  /// \return WorldStatistics message.
  gz::msgs::WorldStatistics Stats() const;

  /// \brief Call a provided function with the EntityComponentManager as the only argument.
  ///
  /// This function locks a mutex before calling the provided function.
  ///
  /// \param[in] f Callback function with the EntityComponentManager as the argument.
  ///
  /// \note The callback function should not call \ref Stats as that function is also thread
  /// synchronized with the same mutex.
  void WithEcm(std::function<void(gz::sim::EntityComponentManager &)> f);

  /// \brief Returns the gz::transport Node
  std::shared_ptr<gz::transport::Node> GzNode();

  /// \brief Check whether the internal state (the local copy of the ECM) has been initialized.
  ///
  /// The internal state is supposed to be initialized the first time the class is instantiated,
  /// however, if the necessary services are not available, it will fail and this function will
  /// return false.
  ///
  /// \return True if the state has been initialized.
  bool StateInitialized() const;

  /// \brief Asserts that the local state (ECM, WorldStatistics) has been updated.
  ///
  /// This waits until a new state message is received with a short timeout period
  /// (kGzServiceTimeoutMs)
  /// \param[out] result Populates the result object with an error code and message if a timeout
  /// occurred.
  /// \return True if the state has been updated before a timeout occurred.
  bool AssertUpdatedState(simulation_interfaces::msg::Result & result);

  /// \brief Asserts that the local state (ECM, WorldStatistics) has been updated.
  ///
  /// This waits until a new world stats message is received with a short timeout period
  /// (kGzServiceTimeoutMs)
  /// \param[out] result Populates the result object with an error code and message if a timeout
  /// occurred.
  /// \return True if the world stats has been updated before a timeout occurred.
  bool AssertUpdatedWorldStats(simulation_interfaces::msg::Result & result);

  /// \brief Wait until simulation reset is detected
  /// \return True if reset was detected.
  bool WaitForResetDetected();

  /// \brief Amount of time to wait for a Gazebo service to become available.
  static constexpr unsigned int kGzServiceTimeoutMs{5000};

  /// \brief Amount of time to wait to receive the latest state update.
  static constexpr unsigned int kGzStateUpdatedTimeoutMs{1000};

private:
  /// \brief Establish initial connection to the Gazebo server.
  ///
  /// This makes the initial service request to Gazebo to receive the name of the world being
  /// simulated. It assumes that there is only one world being simulated. If there are multiple
  /// worlds, it will pick the first.
  ///
  /// \return True if the service request succeeded.
  bool InitializeGazeboConnection();

  /// \brief Wait for critical Gazebo services to become available.
  /// \return True if all the required Gazebo services become available before the service timeout
  /// occurred.
  bool WaitForCriticalServices();

  /// \brief Wait for updated state (ECM, WorldStatistics).
  /// \param[in] timeout Amount of time to wait before giving up.
  /// \return True if the state was updated before a timeout occurred.
  bool WaitForUpdatedState(
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(
      kGzStateUpdatedTimeoutMs));

  /// \brief Callback for receiving SerializedStepMap
  ///
  /// This updates the local ECM and WorldStatistics objects.
  /// \param[in] msg SerializedStepMap message
  void UpdateStateFromMsg(const gz::msgs::SerializedStepMap & msg);

  /// \brief Handle newly created entities
  /// For example, this is responsible for creating necessary ECM Components on newly created
  /// entities, so that the Physics system can populate them.
  void HandleNewEntities();

  /// \brief Initialize all canonical link entities by creating necessary ECM components on them.
  void InitializeAllCanonicalLinks();

  /// \brief Initialize canonical link entities by creating necessary ECM components on them.
  /// \param[in] canonicalLinkEntities A set of canonical links to initialize.
  void InitializeCanonicalLinks(const std::unordered_set<gz::sim::Entity> & canonicalLinkEntities);

  /// \brief Subscribe to a Gazebo topic and store the subscription handler so that topics are
  /// automatically unsubscribed during destruction.
  /// \tparam[in] Args Arguments to be passed to gz::transport::Node::CreateSubscriber
  /// \param[in] topic Subscription topic
  template<typename ... Args>
  void SubscribeToGzTopic(const std::string & topic, Args &&... args)
  {
    auto subscription = this->gz_node_->CreateSubscriber(topic, std::forward<Args>(args)...);
    if (!subscription) {
      RCLCPP_ERROR_STREAM(
        this->ros_node_->get_logger(), "Subscribing to the topic [" << topic << "] failed");
    } else {
      this->gz_subscribers[topic] = std::move(subscription);
    }
  }

  /// \brief Name of Gazebo world being simulated
  std::string world_name_;

  /// \brief ROS node
  std::shared_ptr<rclcpp::Node> ros_node_;

  /// \brief Gazebo Node
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief Mutex used to synchronize access to ecm_ and state_updated_.
  mutable std::mutex state_sync_mutex_;

  /// \brief Local copy of the EntityComponentManager synchronized with the server.
  gz::sim::EntityComponentManager ecm_;

  /// \brief Local copy of the latest WorldStatistics message
  gz::msgs::WorldStatistics world_stats_;

  /// \brief Conditional variable used for waiting on the updated world_stats message.
  std::condition_variable world_stats_cv_;

  /// \brief Used as predicate for waiting on the latest state update with a conditional variable.
  bool state_updated_{false};

  /// \brief Mutex used to synchronize access to world_stats_, and world_stats_updated_.
  mutable std::mutex world_stats_sync_mutex_;

  /// \brief Used as predicate for waiting on the latest world stats update with a conditional
  /// variable.
  bool world_stats_updated_{false};

  /// \brief Conditional variable used for waiting on the latest state update.
  std::condition_variable state_cv_;

  /// \brief Records whether the state has been initialized when this class was first instantiated.
  bool state_intialized_{false};

  /// \brief Holds the future returned by a std::async call made when a sim reset occurs.
  std::future<void> initialize_canonical_links_;

  /// \brief Whether simulation reset was detected
  bool reset_detected_{false};

  /// \brief Mutex to synchronize access to reset_detected_
  mutable std::mutex reset_detected_mutex_;

  /// \brief Conditional variable used for waiting on reset detected.
  std::condition_variable reset_detected_cv_;

  /// \brief gz-transport subscription handles
  /// \TODO(azeey): Using a std::map here instead of a std::vector due to a bug in gz-transport
  /// See https://github.com/gazebosim/gz-transport/issues/717
  std::map<std::string, gz::transport::Node::Subscriber> gz_subscribers;
};
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif  // GZ_SIMULATION_INTERFACES__GAZEBO_PROXY_HPP_
