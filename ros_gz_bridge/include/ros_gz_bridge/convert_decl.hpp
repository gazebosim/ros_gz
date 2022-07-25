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

#ifndef ROS_GZ_BRIDGE__CONVERT_DECL_HPP_
#define ROS_GZ_BRIDGE__CONVERT_DECL_HPP_

namespace ros_gz_bridge
{

template<typename ROS_T, typename GZ_T>
void
convert_ros_to_gz(
  const ROS_T & ros_msg,
  GZ_T & gz_msg);

template<typename ROS_T, typename GZ_T>
void
convert_gz_to_ros(
  const GZ_T & gz_msg,
  ROS_T & ros_msg);

}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT_DECL_HPP_
