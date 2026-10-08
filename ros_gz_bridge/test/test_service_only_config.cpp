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

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace
{
class TestRosGzBridge : public ros_gz_bridge::RosGzBridge
{
public:
  explicit TestRosGzBridge(const rclcpp::NodeOptions & options)
  : RosGzBridge(options)
  {}

  void spin_once()
  {
    spin();
  }

  size_t service_count() const
  {
    return services_.size();
  }
};

class ServiceOnlyConfigTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(ServiceOnlyConfigTest, ServiceOnlyConfigIsLoadedOnce)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("config_file", "test/config/service_only.yaml"),
      rclcpp::Parameter("subscription_heartbeat", 60000)
  });
  auto bridge = std::make_shared<TestRosGzBridge>(options);

  bridge->spin_once();
  ASSERT_EQ(1u, bridge->service_count());

  bridge->spin_once();
  EXPECT_EQ(1u, bridge->service_count());
}
}  // namespace
