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

#include "factories/builtin_interfaces.hpp"
#include "factories/geometry_msgs.hpp"
#include "factories/nav_msgs.hpp"
#include "factories/ros_gz_interfaces.hpp"
#include "factories/rosgraph_msgs.hpp"
#include "factories/sensor_msgs.hpp"
#include "factories/std_msgs.hpp"
#include "factories/tf2_msgs.hpp"
#include "factories/trajectory_msgs.hpp"

#include "service_factories/ros_gz_interfaces.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_impl(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> impl;
  impl = get_factory__builtin_interfaces(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__std_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__geometry_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__nav_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__ros_gz_interfaces(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__rosgraph_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__sensor_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__tf2_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  impl = get_factory__trajectory_msgs(ros_type_name, ign_type_name);
  if (impl) {return impl;}

  return nullptr;
}

std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> factory;
  factory = get_factory_impl(ros_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  throw std::runtime_error("No template specialization for the pair");
}

std::shared_ptr<ServiceFactoryInterface>
get_service_factory(
  const std::string & ros_type_name,
  const std::string & ign_req_type_name,
  const std::string & ign_rep_type_name)
{
  std::shared_ptr<ServiceFactoryInterface> impl;

  impl = get_service_factory__ros_gz_interfaces(
    ros_type_name, ign_req_type_name, ign_rep_type_name);
  if (impl) {return impl;}

  std::ostringstream oss{"No template specialization for the specified service type {"};
  oss << ros_type_name << "}, ign request type {" << ign_req_type_name
      << "}, ign request type {" << ign_req_type_name << "}, ign reply type name {"
      << ign_rep_type_name << "}";
  throw std::runtime_error(oss.str());
}

}  // namespace ros_gz_bridge
