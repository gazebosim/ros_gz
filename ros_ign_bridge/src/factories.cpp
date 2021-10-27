// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

#include "factories.hpp"

#include "factories/geometry_msgs.hpp"
#include "factories/nav_msgs.hpp"
#include "factories/rosgraph_msgs.hpp"
#include "factories/sensor_msgs.hpp"
#include "factories/std_msgs.hpp"
#include "factories/visualization_msgs.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros_type_name,
            const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> factory;

  factory = get_factory_geometry_msgs(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  factory = get_factory_nav_msgs(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  factory = get_factory_rosgraph_msgs(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  factory = get_factory_sensor_msgs(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  factory = get_factory_std_msgs(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  factory = get_factory_visualization_msgs(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  throw std::runtime_error("No template specialization for the pair");
}

}  // namespace ros_ign_bridge
