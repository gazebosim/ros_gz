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

#include <string>
#include <thread>
#include <vector>

#include "convert/logging.hpp"

using ros_gz_bridge::ConvertLogThrottle;
using ros_gz_bridge::kConvertLogMaxMessages;
using ros_gz_bridge::kConvertLogThrottle;

namespace
{
const ConvertLogThrottle::Clock::time_point kStart{};
}  // namespace

TEST(ConvertLogThrottle, RepeatedMessageIsThrottled)
{
  ConvertLogThrottle throttle;
  EXPECT_TRUE(throttle.ShouldLog("a", kStart));
  EXPECT_FALSE(throttle.ShouldLog("a", kStart + kConvertLogThrottle / 2));
  EXPECT_TRUE(throttle.ShouldLog("a", kStart + kConvertLogThrottle));
}

// A problem on one topic doesn't hide a different problem on another topic.
TEST(ConvertLogThrottle, DistinctMessagesAreThrottledSeparately)
{
  ConvertLogThrottle throttle;
  EXPECT_TRUE(throttle.ShouldLog("Unsupported pixel format [a]", kStart));
  EXPECT_TRUE(throttle.ShouldLog("Unsupported pixel format [b]", kStart));
  EXPECT_FALSE(throttle.ShouldLog("Unsupported pixel format [a]", kStart));
  EXPECT_FALSE(throttle.ShouldLog("Unsupported pixel format [b]", kStart));
}

TEST(ConvertLogThrottle, MemoryIsBounded)
{
  ConvertLogThrottle throttle;
  for (std::size_t i = 0; i < 3 * kConvertLogMaxMessages; ++i) {
    EXPECT_TRUE(throttle.ShouldLog(std::to_string(i), kStart));
    EXPECT_LE(throttle.Size(), kConvertLogMaxMessages);
  }
}

// When full, messages that can be logged again are forgotten first.
TEST(ConvertLogThrottle, ExpiredMessagesAreForgottenFirst)
{
  ConvertLogThrottle throttle;
  EXPECT_TRUE(throttle.ShouldLog("recent", kStart + kConvertLogThrottle));
  for (std::size_t i = 1; i < kConvertLogMaxMessages; ++i) {
    EXPECT_TRUE(throttle.ShouldLog(std::to_string(i), kStart));
  }
  ASSERT_EQ(kConvertLogMaxMessages, throttle.Size());

  EXPECT_TRUE(throttle.ShouldLog("new", kStart + kConvertLogThrottle));
  EXPECT_EQ(2u, throttle.Size());
  EXPECT_FALSE(throttle.ShouldLog("recent", kStart + kConvertLogThrottle));
}

TEST(ConvertLogThrottle, ConcurrentCallsLogEachMessageOnce)
{
  ConvertLogThrottle throttle;
  std::vector<int> logged(8, 0);
  std::vector<std::thread> threads;
  for (std::size_t t = 0; t < logged.size(); ++t) {
    threads.emplace_back(
      [&throttle, &logged, t] {
        for (int i = 0; i < 1000; ++i) {
          if (throttle.ShouldLog("shared", kStart)) {
            ++logged[t];
          }
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  int total = 0;
  for (const auto count : logged) {
    total += count;
  }
  EXPECT_EQ(1, total);
}
