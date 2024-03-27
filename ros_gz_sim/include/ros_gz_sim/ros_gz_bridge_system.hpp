// Copyright 2024 Open Source Robotics Foundation
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

#ifndef ROS_GZ_SIM__ROS_GZ_BRIDGE_SYSTEM_HPP_
#define ROS_GZ_SIM__ROS_GZ_BRIDGE_SYSTEM_HPP_

#include <memory>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace ros_gz_sim
{
// Private class.
class ROSGzBridgeSystemPrivate;

/// Important: This system, although functional, it's still under development.
/// The API or its parameters can change in the near future.
/// \brief A ROS 2 <--> gz bridge system.
///
/// This system can be configured with the following SDF parameters:
/// * Optional parameters:
///    * <config_file>: Path to a YAML file containing the list of topics to be
///                     bridged.
class ROSGzBridgeSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  // \brief Constructor.
  ROSGzBridgeSystem();

  /// \brief Destructor.
  ~ROSGzBridgeSystem() override;

  // Documentation inherited.
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  /// \brief Private data pointer.
  std::unique_ptr<ROSGzBridgeSystemPrivate> dataPtr;
};
}  // namespace ros_gz_sim
#endif  // ROS_GZ_SIM__ROS_GZ_BRIDGE_SYSTEM_HPP_
