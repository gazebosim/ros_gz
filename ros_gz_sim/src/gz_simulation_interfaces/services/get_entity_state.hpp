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

#ifndef GZ_SIMULATION_INTERFACES__SERVICES__GET_ENTITY_STATE_HPP_
#define GZ_SIMULATION_INTERFACES__SERVICES__GET_ENTITY_STATE_HPP_

#include <gz/msgs/details/world_stats.pb.h>

#include <memory>
#include <string>

#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <simulation_interfaces/msg/entity_state.hpp>
#include <simulation_interfaces/msg/result.hpp>

#include "../handler_base.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{

class GazeboProxy;

namespace services
{
/// \class GetEntityState
/// \brief Implements the `simulation_interfaces/GetEntityState` interface.
class GetEntityState : public HandlerBase
{
public:
  // Documentation inherited
  GetEntityState(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<GazeboProxy> gz_proxy);

  /// \brief Populate an EntityState object based on our internal state (ECM, WorldStatistics).
  /// \param[in] ecm Reference to the EntityComponentManager.
  /// \param[in] stats Reference to the local copy of the WorldStatistics message.
  /// \param[in] name Name of the entity
  /// \param[out] state State object to be populated.
  static simulation_interfaces::msg::Result FromEcm(
    const gz::sim::EntityComponentManager & ecm, const gz::msgs::WorldStatistics & stats,
    const std::string & name, simulation_interfaces::msg::EntityState & state);

  /// \brief Populate an EntityState object based on our internal state (ECM, WorldStatistics).
  /// \param[in] ecm Reference to the EntityComponentManager.
  /// \param[in] stats Reference to the local copy of the WorldStatistics message.
  /// \param[in] entity Entity ID of the entity whose state is fetched.
  /// \param[out] state State object to be populated.
  static simulation_interfaces::msg::Result FromEcm(
    const gz::sim::EntityComponentManager & ecm, const gz::msgs::WorldStatistics & stats,
    const gz::sim::Entity & entity, simulation_interfaces::msg::EntityState & state);
};
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif  // GZ_SIMULATION_INTERFACES__SERVICES__GET_ENTITY_STATE_HPP_
