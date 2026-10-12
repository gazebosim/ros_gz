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
class TestBridge : public ros_gz_bridge::RosGzBridge
{
public:
  // Release a leaked node after a failed assertion so the test still tears down safely.
  void ClearForCleanup() {handles_.clear();}
};
}  // namespace

class NodeLifetimeTest : public ::testing::TestWithParam<ros_gz_bridge::BridgeDirection>
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_P(NodeLifetimeTest, ReleasesNodeWithActiveSubscriber)
{
  auto bridge = std::make_shared<TestBridge>();
  ros_gz_bridge::BridgeConfig config;
  config.ros_topic_name = "/node_lifetime";
  config.gz_topic_name = "/node_lifetime";
  config.ros_type_name = "std_msgs/msg/String";
  config.gz_type_name = "gz.msgs.StringMsg";
  config.direction = GetParam();
  config.is_lazy = false;
  bridge->add_bridge(config);

  std::weak_ptr<TestBridge> lifetime = bridge;
  bridge.reset();
  EXPECT_TRUE(lifetime.expired()) << "Subscription callback retained the bridge node";
  if (auto leaked = lifetime.lock()) {
    leaked->ClearForCleanup();
  }
}

INSTANTIATE_TEST_SUITE_P(
  SubscriberDirections, NodeLifetimeTest,
  ::testing::Values(
    ros_gz_bridge::BridgeDirection::GZ_TO_ROS,
    ros_gz_bridge::BridgeDirection::ROS_TO_GZ,
    ros_gz_bridge::BridgeDirection::BIDIRECTIONAL));
