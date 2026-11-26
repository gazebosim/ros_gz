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

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include "gz_simulation_interfaces/stop_signal.hpp"

using namespace std::chrono_literals;
using ros_gz_sim::gz_simulation_interfaces::StopSignal;
using WaitResult = StopSignal::WaitResult;
using Clock = std::chrono::steady_clock;

namespace
{
// Generous bound for "woke up promptly" that still is far below the waits' deadlines.
constexpr auto kPrompt = 1s;
constexpr auto kLongWait = 30s;
}  // namespace

TEST(StopSignal, RequestWakesUpWaiter)
{
  StopSignal signal;
  auto waiter = std::async(
    std::launch::async, [&signal] {
      return signal.WaitUntil(Clock::now() + kLongWait, [] {return false;});
    });
  std::this_thread::sleep_for(50ms);  // Let the waiter block.

  const auto start = Clock::now();
  signal.Request();
  ASSERT_EQ(waiter.wait_for(kPrompt), std::future_status::ready);
  EXPECT_EQ(waiter.get(), WaitResult::kStopped);
  EXPECT_LT(Clock::now() - start, kPrompt);
  EXPECT_TRUE(signal.Requested());
}

TEST(StopSignal, UpdateAndNotifyWakesUpWaiter)
{
  StopSignal signal;
  bool done = false;  // Only accessed with the signal's mutex held.
  auto waiter = std::async(
    std::launch::async, [&] {
      return signal.WaitUntil(Clock::now() + kLongWait, [&done] {return done;});
    });
  std::this_thread::sleep_for(50ms);

  signal.UpdateAndNotify([&done] {done = true;});
  ASSERT_EQ(waiter.wait_for(kPrompt), std::future_status::ready);
  EXPECT_EQ(waiter.get(), WaitResult::kReady);
  EXPECT_FALSE(signal.Requested());
}

TEST(StopSignal, TimesOut)
{
  StopSignal signal;
  const auto start = Clock::now();
  EXPECT_EQ(signal.WaitUntil(start + 100ms, [] {return false;}), WaitResult::kTimeout);
  EXPECT_GE(Clock::now() - start, 100ms);
  EXPECT_FALSE(signal.SleepUntil(Clock::now() + 10ms));
}

TEST(StopSignal, ReadyWinsOverStop)
{
  StopSignal signal;
  signal.Request();
  EXPECT_EQ(signal.WaitUntil(Clock::now() + kLongWait, [] {return true;}), WaitResult::kReady);
}

TEST(StopSignal, ReturnsImmediatelyOnceStopped)
{
  StopSignal signal;
  signal.Request();
  const auto start = Clock::now();
  EXPECT_TRUE(signal.SleepUntil(start + kLongWait));
  EXPECT_LT(Clock::now() - start, kPrompt);
}
