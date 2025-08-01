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

#ifndef ROS_GZ_SIM__SIMULATION_INTERFACES_GET_ENTITY_STATE_SERVICE_HPP_
#define ROS_GZ_SIM__SIMULATION_INTERFACES_GET_ENTITY_STATE_SERVICE_HPP_

#include <memory>
#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <simulation_interfaces/msg/entity_state.hpp>
#include <simulation_interfaces/msg/result.hpp>
#include <string>

#include "../handler_base.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{

class GazeboProxy;

namespace services
{
// TODO(azeey) Add documentation
class GetEntityState : public HandlerBase
{
public:
  GetEntityState(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<GazeboProxy> gz_proxy);

  static simulation_interfaces::msg::Result FromEcm(
    const gz::sim::EntityComponentManager & ecm, const std::string & name,
    simulation_interfaces::msg::EntityState & state);

  static simulation_interfaces::msg::Result FromEcm(
    const gz::sim::EntityComponentManager & ecm, const gz::sim::Entity & entity,
    simulation_interfaces::msg::EntityState & state);
};
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif
