/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef ROS_GZ_PLUGIN_HH_
#define ROS_GZ_PLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace ros_gz_sim
{
/// \brief ToDo
class ROSGzPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure
{
  // \brief Constructor.
  public: ROSGzPlugin();

  /// \brief Destructor.
  public: ~ROSGzPlugin() override;

  // Documentation inherited.
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  /// \brief Private data pointer.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}
#endif
