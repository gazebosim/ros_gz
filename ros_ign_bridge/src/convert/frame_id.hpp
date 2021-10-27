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

#ifndef ROS_IGN_BRIDGE__CONVERT__FRAME_ID_HPP__
#define ROS_IGN_BRIDGE__CONVERT__FRAME_ID_HPP__

#include <string>

namespace ros_ign_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(const std::string &input,
                              const std::string &old_delim,
                              const std::string new_delim);

// Frame id from ROS to ign is not supported right now
// std::string frame_id_ros_to_ign(const std::string &frame_id);

std::string frame_id_ign_to_ros(const std::string &frame_id);

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__CONVERT__FRAME_ID_HPP__

