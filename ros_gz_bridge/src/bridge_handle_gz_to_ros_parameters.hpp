// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef BRIDGE_HANDLE_GZ_TO_ROS_PARAMETERS_HPP_
#define BRIDGE_HANDLE_GZ_TO_ROS_PARAMETERS_HPP_

namespace ros_gz_bridge
{

struct BridgeHandleGzToRosParameters {

  /// \brief Override the header.stamp field of the outgoing messages with
  /// the wall time
  bool override_timestamps_with_wall_time = false;

  /// \brief True to publish Image and CameraInfo messages in ROS
  /// z-forward optical frame.
  bool publish_optical_frame = false;
};

}  // namespace ros_gz_bridge

#endif  // BRIDGE_HANDLE_GZ_TO_ROS_PARAMETERS_HPP_
