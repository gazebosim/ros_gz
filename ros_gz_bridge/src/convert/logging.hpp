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

#include <chrono>
#include <cstddef>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ros_gz_bridge
{

/// \brief Logger for conversion problems.
inline rclcpp::Logger convert_logger()
{
  return rclcpp::get_logger("ros_gz_bridge");
}

/// \brief Minimum time between two logs of the same message.
constexpr std::chrono::milliseconds kConvertLogThrottle{5000};

/// \brief Maximum number of distinct messages whose last log time is kept.
constexpr std::size_t kConvertLogMaxMessages{128};

/// \brief Throttles conversion problems, which would otherwise be logged for
/// every message, possibly at camera rates.
///
/// Each distinct message is throttled on its own, so that a problem on one
/// topic doesn't hide a different problem on another topic. Conversions run
/// on gz-transport and ROS threads, so this is thread safe.
class ConvertLogThrottle
{
public:
  using Clock = std::chrono::steady_clock;

  /// \brief Whether a message should be logged now.
  /// \param[in] message Formatted message.
  /// \param[in] now Current time.
  /// \return True if the message wasn't logged in the last kConvertLogThrottle.
  bool ShouldLog(const std::string & message, Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    auto it = this->last_logged_.find(message);
    if (it != this->last_logged_.end()) {
      if (now - it->second < kConvertLogThrottle) {
        return false;
      }
      it->second = now;
      return true;
    }
    if (this->last_logged_.size() >= kConvertLogMaxMessages) {
      // Keep memory bounded: forget the messages that can be logged again
      // anyway, or everything if they are all recent.
      for (auto entry = this->last_logged_.begin(); entry != this->last_logged_.end(); ) {
        entry = (now - entry->second >= kConvertLogThrottle) ?
          this->last_logged_.erase(entry) : std::next(entry);
      }
      if (this->last_logged_.size() >= kConvertLogMaxMessages) {
        this->last_logged_.clear();
      }
    }
    this->last_logged_.emplace(message, now);
    return true;
  }

  /// \brief Number of messages whose last log time is kept.
  std::size_t Size()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->last_logged_.size();
  }

private:
  /// \brief Protects last_logged_.
  std::mutex mutex_;

  /// \brief Last time each message was logged.
  std::unordered_map<std::string, Clock::time_point> last_logged_;
};

/// \brief Throttle shared by all conversions in the process.
inline ConvertLogThrottle & convert_log_throttle()
{
  static ConvertLogThrottle throttle;
  return throttle;
}

}  // namespace ros_gz_bridge

/// \brief Log a conversion problem, at most once every kConvertLogThrottle for
/// each distinct message.
#define ROS_GZ_BRIDGE_CONVERT_LOG_(level_macro, stream_arg) \
  do { \
    std::stringstream ros_gz_bridge_convert_ss_; \
    ros_gz_bridge_convert_ss_ << stream_arg; \
    const std::string ros_gz_bridge_convert_msg_ = ros_gz_bridge_convert_ss_.str(); \
    if (::ros_gz_bridge::convert_log_throttle().ShouldLog(ros_gz_bridge_convert_msg_)) { \
      level_macro( \
        ::ros_gz_bridge::convert_logger(), "%s", ros_gz_bridge_convert_msg_.c_str()); \
    } \
  } while (0)

/// \brief Log a conversion error, at most once every kConvertLogThrottle for
/// each distinct message.
/// \param[in] stream_arg Stream expression, as for RCLCPP_ERROR_STREAM.
#define ROS_GZ_BRIDGE_CONVERT_ERROR(stream_arg) \
  ROS_GZ_BRIDGE_CONVERT_LOG_(RCLCPP_ERROR, stream_arg)

/// \brief Log a conversion warning, at most once every kConvertLogThrottle for
/// each distinct message.
/// \param[in] stream_arg Stream expression, as for RCLCPP_WARN_STREAM.
#define ROS_GZ_BRIDGE_CONVERT_WARN(stream_arg) \
  ROS_GZ_BRIDGE_CONVERT_LOG_(RCLCPP_WARN, stream_arg)

#endif  // CONVERT__LOGGING_HPP_
