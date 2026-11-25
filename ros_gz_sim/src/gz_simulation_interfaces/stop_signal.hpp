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

#ifndef GZ_SIMULATION_INTERFACES__STOP_SIGNAL_HPP_
#define GZ_SIMULATION_INTERFACES__STOP_SIGNAL_HPP_

#include <condition_variable>
#include <mutex>
#include <utility>

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
/// \class StopSignal
/// \brief Shutdown request that wakes up waiting threads immediately.
///
/// Threads block in WaitUntil() for a condition, a deadline or a shutdown request, whichever comes
/// first. Producers of the condition update the state it reads through UpdateAndNotify(), so that
/// the update and the wake-up cannot race with a waiter that is about to block.
class StopSignal
{
public:
  /// \brief Result of WaitUntil().
  enum class WaitResult
  {
    /// \brief The predicate became true.
    kReady,
    /// \brief The deadline passed before the predicate became true.
    kTimeout,
    /// \brief Shutdown was requested before the predicate became true.
    kStopped,
  };

  /// \brief Request shutdown and wake up all waiting threads.
  void Request()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->stopped_ = true;
    }
    this->cv_.notify_all();
  }

  /// \brief Check whether shutdown has been requested.
  /// \return True if Request() has been called.
  bool Requested() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->stopped_;
  }

  /// \brief Update state read by WaitUntil() predicates and wake up all waiting threads.
  /// \param[in] update Callable invoked with the internal mutex held.
  template<typename F>
  void UpdateAndNotify(F && update)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      std::forward<F>(update)();
    }
    this->cv_.notify_all();
  }

  /// \brief Block until `ready()` holds, `deadline` passes, or shutdown is requested.
  ///
  /// `ready` is evaluated with the internal mutex held, so the state it reads must only be modified
  /// through UpdateAndNotify().
  /// \param[in] deadline Time point after which to give up.
  /// \param[in] ready Predicate that ends the wait when it returns true.
  /// \return kReady if `ready()` holds (even if shutdown was also requested), kStopped if shutdown
  /// was requested, kTimeout otherwise.
  template<typename TimePoint, typename Predicate>
  WaitResult WaitUntil(const TimePoint & deadline, Predicate ready)
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    this->cv_.wait_until(lock, deadline, [&] {return this->stopped_ || ready();});
    if (ready()) {
      return WaitResult::kReady;
    }
    return this->stopped_ ? WaitResult::kStopped : WaitResult::kTimeout;
  }

  /// \brief Block until `deadline` passes or shutdown is requested.
  /// \param[in] deadline Time point after which to stop waiting.
  /// \return True if shutdown was requested.
  template<typename TimePoint>
  bool SleepUntil(const TimePoint & deadline)
  {
    return this->WaitUntil(deadline, [] {return false;}) == WaitResult::kStopped;
  }

private:
  /// \brief Protects stopped_ and the state read by WaitUntil() predicates.
  mutable std::mutex mutex_;

  /// \brief Signaled on Request() and UpdateAndNotify().
  std::condition_variable cv_;

  /// \brief Whether shutdown has been requested.
  bool stopped_{false};
};
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim

#endif  // GZ_SIMULATION_INTERFACES__STOP_SIGNAL_HPP_
