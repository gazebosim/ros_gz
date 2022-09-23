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

#include <chrono>
#include <limits>
#include <utility>

#include <ros_gz_sim/Stopwatch.hpp>
#include <rclcpp/clock.hpp>


namespace ros_gz_sim
{

const rclcpp::Duration duration_zero = rclcpp::Duration(0, 0U);
rclcpp::Time time_min = rclcpp::Time(0, 0U);

// Private data class
class StopwatchPrivate
{
public:
  /// \brief Default constructor.
  StopwatchPrivate() = default;

public:
  /// \brief Copy constructor.
  /// \param[in] _watch Watch to copy.
  explicit StopwatchPrivate(const StopwatchPrivate & _watch)
  : running(_watch.running),
    startTime(_watch.startTime),
    stopTime(_watch.stopTime),
    stopDuration(_watch.stopDuration),
    runDuration(_watch.runDuration)
  {
    SetClock(_watch.clock);
  }

public:
  void SetClock(rclcpp::Clock::SharedPtr _clock)
  {
    clock = _clock;
    if (startTime.get_clock_type() != clock->get_clock_type()) {
      startTime = rclcpp::Time(startTime.nanoseconds(), clock->get_clock_type());
    }
    if (stopTime.get_clock_type() != clock->get_clock_type()) {
      stopTime = rclcpp::Time(stopTime.nanoseconds(), clock->get_clock_type());
    }
    if (time_min.get_clock_type() != clock->get_clock_type()) {
      time_min = rclcpp::Time(0U, clock->get_clock_type());
    }
  }

public:
  /// \brief True if the real time clock is running.
  bool running = false;

public:
  /// \brief Time point that marks the start of the real-time clock.
  rclcpp::Time startTime = time_min;

public:
  /// \brief Time point that marks the stop of the real-time clock.
  rclcpp::Time stopTime = time_min;

public:
  /// \brief Amount of stop time.
  rclcpp::Duration stopDuration = duration_zero;

public:
  /// \brief Amount of run time.
  rclcpp::Duration runDuration = duration_zero;

public:
  /// \brief ros clock instance
  rclcpp::Clock::SharedPtr clock = nullptr;
};

//////////////////////////////////////////////////
Stopwatch::Stopwatch()
: dataPtr(new StopwatchPrivate)
{
}

//////////////////////////////////////////////////
Stopwatch::Stopwatch(const Stopwatch & _watch)
: dataPtr(new StopwatchPrivate(*_watch.dataPtr))
{
}

//////////////////////////////////////////////////
Stopwatch::Stopwatch(Stopwatch && _watch) noexcept
: dataPtr(std::move(_watch.dataPtr))
{
}

//////////////////////////////////////////////////
Stopwatch::~Stopwatch()
{
}

//////////////////////////////////////////////////
void Stopwatch::SetClock(rclcpp::Clock::SharedPtr _clock)
{
  this->dataPtr->SetClock(_clock);
}

//////////////////////////////////////////////////
bool Stopwatch::Start(const bool _reset)
{
  if (!this->dataPtr->clock) {
    return false;
  }

  if (_reset) {
    this->Reset();
  }

  if (!this->dataPtr->running) {
    if (this->dataPtr->startTime != this->dataPtr->stopTime) {
      this->dataPtr->stopDuration = this->dataPtr->stopDuration + \
        this->dataPtr->clock->now() - this->dataPtr->stopTime;
    }

    this->dataPtr->running = true;
    this->dataPtr->startTime = this->dataPtr->clock->now();
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
const rclcpp::Time & Stopwatch::StartTime() const
{
  return this->dataPtr->startTime;
}

//////////////////////////////////////////////////
bool Stopwatch::Stop()
{
  if (!this->dataPtr->clock) {
    return false;
  }

  if (this->dataPtr->running) {
    this->dataPtr->running = false;
    this->dataPtr->stopTime = this->dataPtr->clock->now();
    this->dataPtr->runDuration = this->dataPtr->runDuration +
      this->dataPtr->stopTime - this->dataPtr->startTime;
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
const rclcpp::Time & Stopwatch::StopTime() const
{
  return this->dataPtr->stopTime;
}

//////////////////////////////////////////////////
bool Stopwatch::Running() const
{
  return this->dataPtr->running;
}

//////////////////////////////////////////////////
void Stopwatch::Reset()
{
  this->dataPtr->running = false;
  this->dataPtr->startTime = time_min;
  this->dataPtr->stopTime = time_min;
  this->dataPtr->stopDuration = duration_zero;
  this->dataPtr->runDuration = duration_zero;
}

//////////////////////////////////////////////////
rclcpp::Duration Stopwatch::ElapsedRunTime() const
{
  if (!this->dataPtr->clock) {
    return duration_zero;
  }

  if (this->dataPtr->running) {
    return this->dataPtr->clock->now() - this->dataPtr->startTime + this->dataPtr->runDuration;
  } else {
    return this->dataPtr->runDuration;
  }
}

//////////////////////////////////////////////////
rclcpp::Duration Stopwatch::ElapsedStopTime() const
{
  if (!this->dataPtr->clock) {
    return duration_zero;
  }

  // If running, then return the stopDuration.
  if (this->dataPtr->running) {
    return this->dataPtr->stopDuration;
  } else if (this->dataPtr->stopTime > time_min) {
    // The clock is not running, and Stop() has been called.
    return this->dataPtr->stopDuration +
           (this->dataPtr->clock->now() - this->dataPtr->stopTime);
  }

  // Otherwise, the stopwatch has been reset or never started.
  return duration_zero;
}

//////////////////////////////////////////////////
bool Stopwatch::operator==(const Stopwatch & _watch) const
{
  return this->dataPtr->running == _watch.dataPtr->running &&
         this->dataPtr->startTime == _watch.dataPtr->startTime &&
         this->dataPtr->stopTime == _watch.dataPtr->stopTime &&
         this->dataPtr->stopDuration == _watch.dataPtr->stopDuration &&
         this->dataPtr->runDuration == _watch.dataPtr->runDuration;
}

//////////////////////////////////////////////////
bool Stopwatch::operator!=(const Stopwatch & _watch) const
{
  return !(*this == _watch);
}

//////////////////////////////////////////////////
Stopwatch & Stopwatch::operator=(const Stopwatch & _watch)
{
  this->dataPtr.reset(new StopwatchPrivate(*_watch.dataPtr));
  return *this;
}

//////////////////////////////////////////////////
Stopwatch & Stopwatch::operator=(Stopwatch && _watch)
{
  this->dataPtr = std::move(_watch.dataPtr);
  return *this;
}
}  // namespace ros_gz_sim
