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

#ifndef ROS_IGN_UTILS__OPTICAL_FRAME_PUBLISHER_HPP_
#define ROS_IGN_UTILS__OPTICAL_FRAME_PUBLISHER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace ros_ign_utils
{

class OpticalFramePublisher : public rclcpp::Node
{
public:
  explicit OpticalFramePublisher(const rclcpp::NodeOptions & options);

private:
  struct Impl;
  std::unique_ptr<Impl> dataPtr;
};

}  // namespace ros_ign_utils

#endif  //  ROS_IGN_UTILS__OPTICAL_FRAME_PUBLISHER_HPP_
