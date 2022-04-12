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

#ifndef ROS_IGN_BRIDGE__ROS_IGN_BRIDGE_HPP_
#define ROS_IGN_BRIDGE__ROS_IGN_BRIDGE_HPP_

#include <ignition/msgs/config.hh>

// Dataframe is available from versions 8.4.0 (fortress) forward
// This can be removed when the minimum supported version passes 8.4.0
#if (IGNITION_MSGS_MAJOR_VERSION >= 8 && \
  IGNITION_MSGS_MINOR_VERSION >= 4 && \
  IGNITION_MSGS_PATCH_VERSION >= 0)
#define HAVE_DATAFRAME true
#endif

#endif  // ROS_IGN_BRIDGE__ROS_IGN_BRIDGE_HPP_
