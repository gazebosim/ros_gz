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

#ifndef GZ_SIMULATION_INTERFACES__HANDLER_BASE_HPP_
#define GZ_SIMULATION_INTERFACES__HANDLER_BASE_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{

class GazeboProxy;


/// \class HandlerBase
/// \brief Base class for Service Interface handlers
///
/// This just holds the ROS and Gazebo nodes as well as the service handles
class HandlerBase
{
public:
  /// \brief Constructor
  /// \param[in] ros_node ROS Node
  /// \param[in] gz_proxy Gazebo Node
  HandlerBase(std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
  : ros_node_(ros_node), gz_proxy_(gz_proxy)
  {
  }

protected:
  /// \brief ROS Node
  std::shared_ptr<rclcpp::Node> ros_node_;

  /// \brief Gazebo Node
  std::shared_ptr<GazeboProxy> gz_proxy_;

  /// \brief ROS Service handle for the service provided by the derived class.
  std::shared_ptr<rclcpp::ServiceBase> services_handle_;
};
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif  // GZ_SIMULATION_INTERFACES__HANDLER_BASE_HPP_
