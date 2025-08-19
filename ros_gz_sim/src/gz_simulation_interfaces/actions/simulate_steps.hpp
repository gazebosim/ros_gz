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

#ifndef ROS_GZ_SIM__SIMULATION_INTERFACES_SIMULATE_STEPS_SERVICE_HPP_
#define ROS_GZ_SIM__SIMULATION_INTERFACES_SIMULATE_STEPS_SERVICE_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "../handler_base.hpp"

namespace ros_gz_sim
{

namespace gz_simulation_interfaces
{
class GazeboProxy;
namespace actions
{

/// \class SimulateSteps
/// \brief Implements the `simulation_interfaces/SimulateSteps` interface.
class SimulateSteps : public HandlerBase
{
public:
  // Documentation inherited
  SimulateSteps(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<GazeboProxy> gz_proxy);

private:
  // TODO(azeey) Refactor into base class
  std::shared_ptr<rclcpp_action::ServerBase> action_handles_;
};
}  // namespace actions
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif
