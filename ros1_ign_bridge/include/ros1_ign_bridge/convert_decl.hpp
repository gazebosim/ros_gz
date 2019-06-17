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

#ifndef ROS1_IGN_BRIDGE__CONVERT_DECL_HPP_
#define ROS1_IGN_BRIDGE__CONVERT_DECL_HPP_

namespace ros1_ign_bridge
{

template<typename ROS1_T, typename IGN_T>
void
convert_1_to_ign(
  const ROS1_T & ros1_msg,
  IGN_T & ign_msg);

template<typename ROS1_T, typename IGN_T>
void
convert_ign_to_1(
  const IGN_T & ign_msg,
  ROS1_T & ros1_msg);

}  // namespace ros1_ign_bridge

#endif  // ROS1_IGN_BRIDGE__CONVERT_DECL_HPP_
