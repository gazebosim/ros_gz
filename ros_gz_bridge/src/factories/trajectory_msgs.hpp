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

#ifndef FACTORIES__TRAJECTORY_MSGS_HPP_
#define FACTORIES__TRAJECTORY_MSGS_HPP_

#include <memory>
#include <string>

#include "factory_interface.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__trajectory_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name);

}  // namespace ros_gz_bridge

#endif  // FACTORIES__TRAJECTORY_MSGS_HPP_
