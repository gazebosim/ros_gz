/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef ROS_IGN_GAZEBO__OPTICAL_FRAME_PUBLISHER_HPP_
#define ROS_IGN_GAZEBO__OPTICAL_FRAME_PUBLISHER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace ros_ign_gazebo
{

/// \brief ROS Node to transform image coordinate conventions to match
/// ROS REP 103 (https://www.ros.org/reps/rep-0103.html)
class OpticalFramePublisher : public rclcpp::Node
{
public:
    /// \param[in] options additional controls for creating the ROS node
  /// \brief Constructor
  explicit OpticalFramePublisher(const rclcpp::NodeOptions & options);

private:
  /// \brief Forward declaration of private implementation
  struct Impl;

  /// \brief Pointer to implementation
  std::unique_ptr<Impl> dataPtr;
};

}  // namespace ros_ign_utils

#endif  //  ROS_IGN_UTILS__OPTICAL_FRAME_PUBLISHER_HPP_
