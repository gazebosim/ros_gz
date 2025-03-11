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

#include <optional>
#include <geometry_msgs/msg/transform.hpp>

namespace ros_gz_bridge
{

struct BridgeHandleGzToRosParameters {
  /// \brief Override the header.stamp field of the outgoing messages with
  /// the wall time
  bool override_timestamps_with_wall_time = false;

  /// Publish messages under a new frame which has the specified transformation
  /// from the original frame. This is done by broadcasting a tf with the
  /// frame_id set to the frame_id in the original message, and the
  /// child_frame_id set to the new frame id in the updated outgoing message.
  /// The new frame id of the outgoing messages must be set via the
  /// override_frame_id_string parameter.
  std::optional<geometry_msgs::msg::Transform> override_frame_transform;

  /// \brief Override the header.frame_id field of the outgoing messages with
  /// this new frame_id string
  std::string override_frame_id_string;

  /// \brief Append the header.frame_id field of the outgoing messages with
  /// a suffix string. Used internally for publishing optical frames.
  std::string override_frame_id_suffix_string;
};

}  // namespace ros_gz_bridge

#endif  // BRIDGE_HANDLE_GZ_TO_ROS_PARAMETERS_HPP_
