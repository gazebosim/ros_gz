// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef CONVERT__LOGGING_HPP_
#define CONVERT__LOGGING_HPP_

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ros_gz_bridge
{

/// \brief Logger for conversion problems.
inline rclcpp::Logger convert_logger()
{
  return rclcpp::get_logger("ros_gz_bridge");
}

/// \brief Clock used to throttle conversion problems, which would otherwise
/// be logged for every message, possibly at camera rates.
inline rclcpp::Clock & convert_log_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

/// \brief Minimum time between two logs from the same place, in milliseconds.
constexpr int kConvertLogThrottleMs = 5000;

}  // namespace ros_gz_bridge

/// \brief Log a conversion error, at most once every kConvertLogThrottleMs.
/// \param[in] stream_arg Stream expression, as for RCLCPP_ERROR_STREAM.
#define ROS_GZ_BRIDGE_CONVERT_ERROR(stream_arg) \
  RCLCPP_ERROR_STREAM_THROTTLE( \
    ::ros_gz_bridge::convert_logger(), ::ros_gz_bridge::convert_log_clock(), \
    ::ros_gz_bridge::kConvertLogThrottleMs, stream_arg)

/// \brief Log a conversion warning, at most once every kConvertLogThrottleMs.
/// \param[in] stream_arg Stream expression, as for RCLCPP_WARN_STREAM.
#define ROS_GZ_BRIDGE_CONVERT_WARN(stream_arg) \
  RCLCPP_WARN_STREAM_THROTTLE( \
    ::ros_gz_bridge::convert_logger(), ::ros_gz_bridge::convert_log_clock(), \
    ::ros_gz_bridge::kConvertLogThrottleMs, stream_arg)

#endif  // CONVERT__LOGGING_HPP_
