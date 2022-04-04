// Copyright 2021 Open Source Robotics Foundation, Inc.
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

// generated from ros_ign_bridge/resource/get_factory.cpp.em

#include <memory>
#include <string>

#include "get_factory.hpp"

@[for ros2_package_name in sorted(ros2_package_names)]@
#include "factories/@(ros2_package_name).hpp"
@[end for]@

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_impl(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> impl = nullptr;
@[for ros2_package_name in sorted(ros2_package_names)]@
  impl = get_factory__@(ros2_package_name)(ros_type_name, ign_type_name);
  if (impl) {return impl;}
@[end for]
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

}  // namespace ros_ign_bridge

