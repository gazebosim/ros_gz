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
#include <gz/msgs/int64.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr auto kDiscoveryTimeout = 5s;

class DynamicBridgeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

protected:
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

protected:
  void SetUp() override
  {
    static std::atomic<int> nodeId{0};
    this->rosNode = std::make_shared<rclcpp::Node>(
      "dynamic_bridge_test_" + std::to_string(nodeId++));
    this->executor.add_node(this->rosNode);
  }

protected:
  void TearDown() override
  {
    this->executor.remove_node(this->rosNode);
    this->rosNode.reset();
  }

protected:
  bool SpinUntil(
    const std::function<bool()> & _condition,
    const std::chrono::steady_clock::duration & _timeout = kDiscoveryTimeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + _timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      this->executor.spin_some();
      if (_condition()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    this->executor.spin_some();
    return _condition();
  }

protected:
  rclcpp::Node::SharedPtr rosNode;

protected:
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(DynamicBridgeTest, DiscoversTopicAfterStartup)
{
  const std::string topic = "/dynamic_bridge_late_topic";
  std::atomic<bool> received{false};
  std::string receivedData;
  auto subscription = this->rosNode->create_subscription<std_msgs::msg::String>(
    topic, 10,
    [&received, &receivedData](const std_msgs::msg::String & _msg)
    {
      receivedData = _msg.data;
      received = true;
    });

  gz::transport::Node gzNode;
  auto publisher = gzNode.Advertise<gz::msgs::StringMsg>(topic);
  ASSERT_TRUE(publisher.Valid());
  ASSERT_TRUE(this->SpinUntil([&publisher]() {return publisher.HasConnections();}));
  ASSERT_TRUE(this->SpinUntil(
      [this, &topic]() {return this->rosNode->count_publishers(topic) >= 1u;}));

  gz::msgs::StringMsg message;
  message.set_data("late topic");
  ASSERT_TRUE(publisher.Publish(message));
  ASSERT_TRUE(this->SpinUntil([&received]() {return received.load();}));
  EXPECT_EQ("late topic", receivedData);
}

TEST_F(DynamicBridgeTest, BridgesBidirectionally)
{
  const std::string topic = "/dynamic_bridge_bidirectional";
  std::mutex mutex;
  std::vector<std::string> rosMessages;
  std::vector<std::string> gzMessages;

  auto rosSubscription = this->rosNode->create_subscription<std_msgs::msg::String>(
    topic, 10,
    [&mutex, &rosMessages](const std_msgs::msg::String & _msg)
    {
      std::lock_guard<std::mutex> lock(mutex);
      rosMessages.push_back(_msg.data);
    });
  auto rosPublisher = this->rosNode->create_publisher<std_msgs::msg::String>(topic, 10);

  gz::transport::Node gzNode;
  auto gzPublisher = gzNode.Advertise<gz::msgs::StringMsg>(topic);
  ASSERT_TRUE(gzPublisher.Valid());
  ASSERT_TRUE(this->SpinUntil([&gzPublisher]() {return gzPublisher.HasConnections();}));
  ASSERT_TRUE(this->SpinUntil(
      [this, &topic]() {return this->rosNode->count_publishers(topic) >= 2u;}));

  gz::msgs::StringMsg gzMessage;
  gzMessage.set_data("from Gazebo");
  ASSERT_TRUE(gzPublisher.Publish(gzMessage));
  ASSERT_TRUE(this->SpinUntil(
      [&mutex, &rosMessages]()
      {
        std::lock_guard<std::mutex> lock(mutex);
        return std::find(rosMessages.begin(), rosMessages.end(), "from Gazebo") !=
               rosMessages.end();
    }));

  std::function<void(const gz::msgs::StringMsg &)> gzCallback =
    [&mutex, &gzMessages](const gz::msgs::StringMsg & _msg)
    {
      std::lock_guard<std::mutex> lock(mutex);
      gzMessages.push_back(_msg.data());
    };
  ASSERT_TRUE(gzNode.Subscribe(topic, gzCallback));

  ASSERT_TRUE(this->SpinUntil(
      [this]() {
        return this->rosNode->count_subscribers(
      "/dynamic_bridge_bidirectional") > 0;
                                           }));

  std_msgs::msg::String rosMessage;
  rosMessage.data = "from ROS";
  ASSERT_TRUE(this->SpinUntil(
      [&mutex, &gzMessages, &rosMessage, &rosPublisher]()
      {
        rosPublisher->publish(rosMessage);
        std::lock_guard<std::mutex> lock(mutex);
        return std::find(gzMessages.begin(), gzMessages.end(), "from ROS") != gzMessages.end();
    }));
}

TEST_F(DynamicBridgeTest, DoesNotCreateDuplicateBridges)
{
  const std::string topic = "/dynamic_bridge_no_duplicates";
  std::atomic<int> received{0};
  auto subscription = this->rosNode->create_subscription<std_msgs::msg::String>(
    topic, 10,
    [&received](const std_msgs::msg::String &) {++received;});

  gz::transport::Node gzNode;
  auto publisher = gzNode.Advertise<gz::msgs::StringMsg>(topic);
  ASSERT_TRUE(publisher.Valid());
  ASSERT_TRUE(this->SpinUntil([&publisher]() {return publisher.HasConnections();}));
  ASSERT_TRUE(this->SpinUntil(
      [this, &topic]()
      {
        return !this->rosNode->get_publishers_info_by_topic(topic).empty();
    }));

  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    this->executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(1u, this->rosNode->get_publishers_info_by_topic(topic).size());

  gz::msgs::StringMsg message;
  message.set_data("once");
  ASSERT_TRUE(publisher.Publish(message));
  ASSERT_TRUE(this->SpinUntil([&received]() {return received.load() > 0;}));
  std::this_thread::sleep_for(100ms);
  this->executor.spin_some();
  EXPECT_EQ(1, received.load());
}

TEST_F(DynamicBridgeTest, IgnoresUnsupportedTypesAndKeepsRunning)
{
  gz::transport::Node gzNode;
  const std::string unsupportedTopic = "/dynamic_bridge_unsupported";
  auto unsupportedPublisher = gzNode.Advertise<gz::msgs::Int64>(unsupportedTopic);
  ASSERT_TRUE(unsupportedPublisher.Valid());

  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    this->executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(this->rosNode->get_publishers_info_by_topic(unsupportedTopic).empty());

  const std::string supportedTopic = "/dynamic_bridge_after_unsupported";
  std::atomic<bool> received{false};
  auto subscription = this->rosNode->create_subscription<std_msgs::msg::String>(
    supportedTopic, 10,
    [&received](const std_msgs::msg::String &) {received = true;});
  auto supportedPublisher = gzNode.Advertise<gz::msgs::StringMsg>(supportedTopic);
  ASSERT_TRUE(supportedPublisher.Valid());
  ASSERT_TRUE(this->SpinUntil(
      [&supportedPublisher]() {return supportedPublisher.HasConnections();}));
  ASSERT_TRUE(this->SpinUntil(
      [this, &supportedTopic]()
      {
        return this->rosNode->count_publishers(supportedTopic) == 1u;
    }));

  gz::msgs::StringMsg message;
  message.set_data("still running");
  ASSERT_TRUE(supportedPublisher.Publish(message));
  EXPECT_TRUE(this->SpinUntil([&received]() {return received.load();}));
}

TEST_F(DynamicBridgeTest, DeliversBurstWithoutDuplicates)
{
  constexpr int messageCount = 50;
  const std::string topic = "/dynamic_bridge_burst";
  std::mutex mutex;
  std::vector<int> received;
  auto subscription = this->rosNode->create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(messageCount),
    [&mutex, &received](const std_msgs::msg::String & _msg)
    {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(std::stoi(_msg.data));
    });

  gz::transport::Node gzNode;
  auto publisher = gzNode.Advertise<gz::msgs::StringMsg>(topic);
  ASSERT_TRUE(publisher.Valid());
  ASSERT_TRUE(this->SpinUntil([&publisher]() {return publisher.HasConnections();}));
  ASSERT_TRUE(this->SpinUntil(
      [this, &topic]() {return this->rosNode->count_publishers(topic) >= 1u;}));

  for (int i = 0; i < messageCount; ++i) {
    gz::msgs::StringMsg message;
    message.set_data(std::to_string(i));
    ASSERT_TRUE(publisher.Publish(message));
    this->executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(this->SpinUntil(
      [&mutex, &received]()
      {
        std::lock_guard<std::mutex> lock(mutex);
        return received.size() == messageCount;
    }));

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(messageCount, received.size());
  EXPECT_EQ(messageCount, std::set<int>(received.begin(), received.end()).size());
  for (int i = 0; i < messageCount; ++i) {
    EXPECT_EQ(i, received[i]);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
