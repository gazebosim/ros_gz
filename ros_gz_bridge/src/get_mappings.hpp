// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef GET_MAPPINGS_HPP_
#define GET_MAPPINGS_HPP_

#include <map>
#include <string>

namespace ros_gz_bridge
{

bool
get_gz_to_ros_mapping(const std::string & gz_type_name, std::string & ros_type_name);

bool
get_ros_to_gz_mapping(const std::string & ros_type_name, std::string & gz_type_name);

std::multimap<std::string, std::string>
get_all_message_mappings_ros_to_gz();

}  // namespace ros_gz_bridge

#endif  // GET_MAPPINGS_HPP_
