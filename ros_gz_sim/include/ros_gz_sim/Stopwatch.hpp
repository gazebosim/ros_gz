// Copyright 2022 Open Source Robotics Foundation
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

#ifndef ROS_GZ_SIM__STOPWATCH_HPP_
#define ROS_GZ_SIM__STOPWATCH_HPP_

#include <chrono>
#include <memory>

#include <rclcpp/clock.hpp>

namespace ros_gz_sim
{
// Forward declarations.
class StopwatchPrivate;

/// \brief The stopwatch accepts an rclcpp::Clock instance from a rclcpp::Node (allowing use of
/// sim time if use_sim_time set in node). Keeps track of time spent in the run state, accessed
/// through ElapsedRunTime(), and time spent in the stop state, accessed through ElapsedStopTime().
/// Elapsed run time starts accumulating after the first call to Start(). Elapsed stop time starts
/// accumulation after Start() has been called followed by Stop(). The stopwatch can be reset with
/// the Reset() function.
///
/// # Example usage
///
/// ```{.cpp}
/// ros_gz_sim::Stopwatch watch;
/// watch.Start();
///
/// // do something...
///
/// std::cout << "Elapsed time is "
/// << std::chrono::duration_cast<std::chrono::milliseconds>(
///   timeSys.ElapsedRunTime()).count() << " ms\n";
/// watch.Stop();
/// ```
class Stopwatch
{
  /// \brief Constructor.

public:
  Stopwatch();

  /// \brief Copy constructor
  /// \param[in] _watch The stop watch to copy.

public:
  Stopwatch(const Stopwatch & _watch);

  /// \brief Move constructor
  /// \param[in] _watch The stop watch to move.

public:
  Stopwatch(Stopwatch && _watch) noexcept;

  /// \brief Destructor.

public:
  virtual ~Stopwatch();

public:
  /// \brief Take a clock instance (e.g. get_clock() from rclcpp::Node).
  /// Can also follow sim time on /clock when node's use_sim_time param is set
  /// param[in] _clock
  void SetClock(rclcpp::Clock::SharedPtr _clock);

  /// \brief Start the stopwatch.
  /// \param[in] _reset If true the stopwatch is reset first.
  /// \return True if the the stopwatch was started. This will return
  /// false if the stopwatch was already running.

public:
  bool Start(const bool _reset = false);

  /// \brief Get the time when the stopwatch was started.
  /// \return The time when stopwatch was started, or
  /// std::chrono::steady_clock::time_point::min() if the stopwatch
  /// has not been started.

public:
  const rclcpp::Time & StartTime() const;

  /// \brief Stop the stopwatch
  /// \return True if the stopwatch was stopped. This will return false
  /// if the stopwatch is not running.

public:
  bool Stop();

  /// \brief Get the time when the stopwatch was last stopped.
  /// \return The time when stopwatch was last stopped, or
  /// std::chrono::steady_clock::time_point::min() if the stopwatch
  /// has never been stopped.

public:
  const rclcpp::Time & StopTime() const;

  /// \brief Get whether the stopwatch is running.
  /// \return True if the stopwatch is running.

public:
  bool Running() const;

  /// \brief Reset the stopwatch. This resets the start time, stop time,
  /// elapsed duration and elapsed stop duration.

public:
  void Reset();

  /// \brief Get the amount of time that the stop watch has been
  /// running. This is the total amount of run time, spannning all start
  /// and stop calls. The Reset function or passing true to the Start
  /// function will reset this value.
  /// \return Total amount of elapsed run time.

public:
  rclcpp::Duration ElapsedRunTime() const;

  /// \brief Get the amount of time that the stop watch has been
  /// stopped. This is the total amount of stop time, spannning all start
  /// and stop calls. The Reset function or passing true to the Start
  /// function will reset this value.
  /// \return Total amount of elapsed stop time.

public:
  rclcpp::Duration ElapsedStopTime() const;

  /// \brief Equality operator.
  /// \param[in] _watch The watch to compare.
  /// \return True if this watch equals the provided watch.

public:
  bool operator==(const Stopwatch & _watch) const;

  /// \brief Inequality operator.
  /// \param[in] _watch The watch to compare.
  /// \return True if this watch does not equal the provided watch.

public:
  bool operator!=(const Stopwatch & _watch) const;

  /// \brief Copy assignment operator
  /// \param[in] _watch The stop watch to copy.
  /// \return Reference to this.

public:
  Stopwatch & operator=(const Stopwatch & _watch);

  /// \brief Move assignment operator
  /// \param[in] _watch The stop watch to move.
  /// \return Reference to this.

public:
  Stopwatch & operator=(Stopwatch && _watch);

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
  /// \brief Private data pointer.

private:
  std::unique_ptr<StopwatchPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
};
}  // namespace ros_gz_sim
#endif  // ROS_GZ_SIM__STOPWATCH_HPP_
