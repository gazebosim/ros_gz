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
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
// Gives the tests control over when the lazy bridges check for subscribers.
class TestBridge : public ros_gz_bridge::RosGzBridge
{
public:
  TestBridge()
  : RosGzBridge(rclcpp::NodeOptions().parameter_overrides({{"subscription_heartbeat", 3600000}}))
  {
  }

  using RosGzBridge::spin;

  /// Whether the bridge's own Gazebo node subscribes to the topic.
  bool GzSubscribes(const std::string & topic) const
  {
    const auto topics = gz_node_->SubscribedTopics();
    return std::find(topics.begin(), topics.end(), topic) != topics.end();
  }
};

ros_gz_bridge::BridgeConfig StringBridge(
  const std::string & ros_topic, const std::string & gz_topic,
  ros_gz_bridge::BridgeDirection direction, bool lazy)
{
  ros_gz_bridge::BridgeConfig config;
  config.ros_type_name = "std_msgs/msg/String";
  config.gz_type_name = "gz.msgs.StringMsg";
  config.ros_topic_name = ros_topic;
  config.gz_topic_name = gz_topic;
  config.direction = direction;
  config.is_lazy = lazy;
  return config;
}

/// Run step until done returns true or the timeout expires.
bool WaitFor(
  const std::function<bool()> & done, const std::function<void()> & step = [] {},
  std::chrono::milliseconds timeout = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    step();
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return done();
}
}  // namespace

class BridgeNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    bridge_ = std::make_shared<TestBridge>();
    test_node_ = std::make_shared<rclcpp::Node>("bridge_node_test");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
  }

  void TearDown() override
  {
    executor_.reset();
    test_node_.reset();
    bridge_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TestBridge> bridge_;
  rclcpp::Node::SharedPtr test_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// A non-lazy bridge subscribes to Gazebo right away.
TEST_F(BridgeNodeTest, NonLazyGzToRosSubscribesImmediately)
{
  bridge_->add_bridge(
    StringBridge(
      "/eager_chatter", "/eager_chatter", ros_gz_bridge::BridgeDirection::GZ_TO_ROS, false));
  EXPECT_TRUE(bridge_->GzSubscribes("/eager_chatter"));
}

// A lazy GZ -> ROS bridge subscribes to Gazebo only while something subscribes
// to its ROS topic, and messages flow while it does.
TEST_F(BridgeNodeTest, LazyGzToRosFollowsRosSubscribers)
{
  bridge_->add_bridge(
    StringBridge(
      "/lazy_gz_to_ros", "/lazy_gz_to_ros", ros_gz_bridge::BridgeDirection::GZ_TO_ROS, true));
  bridge_->spin();
  EXPECT_FALSE(bridge_->GzSubscribes("/lazy_gz_to_ros"));

  std::atomic<bool> received{false};
  auto sub = test_node_->create_subscription<std_msgs::msg::String>(
    "/lazy_gz_to_ros", 10, [&received](const std_msgs::msg::String & msg) {
      received = msg.data == "hello";
    });
  ASSERT_TRUE(
    WaitFor(
      [this] {return bridge_->GzSubscribes("/lazy_gz_to_ros");},
      [this] {bridge_->spin();}));

  gz::transport::Node gz_node;
  auto gz_pub = gz_node.Advertise<gz::msgs::StringMsg>("/lazy_gz_to_ros");
  gz::msgs::StringMsg gz_msg;
  gz_msg.set_data("hello");
  EXPECT_TRUE(
    WaitFor(
      [&received] {return received.load();},
      [&] {
        gz_pub.Publish(gz_msg);
        executor_->spin_some();
      }));

  sub.reset();
  EXPECT_TRUE(
    WaitFor(
      [this] {return !bridge_->GzSubscribes("/lazy_gz_to_ros");},
      [this] {bridge_->spin();}));
}

// A lazy ROS -> GZ bridge subscribes to ROS only while something subscribes
// to its Gazebo topic.
TEST_F(BridgeNodeTest, LazyRosToGzFollowsGzSubscribers)
{
  bridge_->add_bridge(
    StringBridge(
      "/lazy_ros_to_gz", "/lazy_ros_to_gz", ros_gz_bridge::BridgeDirection::ROS_TO_GZ, true));
  bridge_->spin();
  EXPECT_EQ(0u, test_node_->count_subscribers("/lazy_ros_to_gz"));

  {
    gz::transport::Node gz_node;
    ASSERT_TRUE(
      gz_node.Subscribe(
        "/lazy_ros_to_gz", std::function<void(const gz::msgs::StringMsg &)>(
          [](const gz::msgs::StringMsg &) {})));
    EXPECT_TRUE(
      WaitFor(
        [this] {return test_node_->count_subscribers("/lazy_ros_to_gz") == 1u;},
        [this] {bridge_->spin();}));
  }

  EXPECT_TRUE(
    WaitFor(
      [this] {return test_node_->count_subscribers("/lazy_ros_to_gz") == 0u;},
      [this] {bridge_->spin();}));
}

// Two bridges reading the same Gazebo topic both deliver every message.
TEST_F(BridgeNodeTest, TwoBridgesOnOneGzTopic)
{
  bridge_->add_bridge(
    StringBridge(
      "/shared_a", "/shared_gz_topic", ros_gz_bridge::BridgeDirection::GZ_TO_ROS, false));
  bridge_->add_bridge(
    StringBridge(
      "/shared_b", "/shared_gz_topic", ros_gz_bridge::BridgeDirection::GZ_TO_ROS, false));

  std::atomic<bool> received_a{false};
  std::atomic<bool> received_b{false};
  auto sub_a = test_node_->create_subscription<std_msgs::msg::String>(
    "/shared_a", 10, [&received_a](const std_msgs::msg::String &) {received_a = true;});
  auto sub_b = test_node_->create_subscription<std_msgs::msg::String>(
    "/shared_b", 10, [&received_b](const std_msgs::msg::String &) {received_b = true;});

  gz::transport::Node gz_node;
  auto gz_pub = gz_node.Advertise<gz::msgs::StringMsg>("/shared_gz_topic");
  gz::msgs::StringMsg gz_msg;
  gz_msg.set_data("shared");
  WaitFor(
    [&] {return received_a && received_b;},
    [&] {
      gz_pub.Publish(gz_msg);
      executor_->spin_some();
    });
  EXPECT_TRUE(received_a);
  EXPECT_TRUE(received_b);
}
