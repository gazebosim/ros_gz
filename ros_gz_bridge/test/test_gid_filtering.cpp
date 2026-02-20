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

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class GidFilteringTest : public ::testing::Test
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

  /// Collect publisher GIDs for a given topic that belong to the specified node.
  /// This replicates the logic in factory.hpp create_ros_subscriber().
  static std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>>
  collect_self_publisher_gids(
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  {
    std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> gids;
    for (const auto & info : node->get_publishers_info_by_topic(topic)) {
      if (info.node_name() == node->get_name() &&
        info.node_namespace() == node->get_namespace())
      {
        gids.push_back(info.endpoint_gid());
      }
    }
    return gids;
  }

  /// Check if a sender GID matches any in a list of GIDs.
  /// This replicates the filtering logic in factory.hpp callback.
  static bool is_from_self(
    const rmw_gid_t & sender_gid,
    const std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> & self_gids)
  {
    for (const auto & gid : self_gids) {
      if (std::memcmp(sender_gid.data, gid.data(), RMW_GID_STORAGE_SIZE) == 0) {
        return true;
      }
    }
    return false;
  }
};

// Verify that GID collection finds publishers from the same node.
TEST_F(GidFilteringTest, CollectsOwnPublisherGids)
{
  auto node = std::make_shared<rclcpp::Node>("bridge_node");
  const std::string topic = "/test_gid_collect";

  auto pub = node->create_publisher<std_msgs::msg::String>(topic, 10);

  // Allow DDS discovery within the same process.
  std::this_thread::sleep_for(100ms);

  auto gids = collect_self_publisher_gids(node, topic);
  EXPECT_EQ(1u, gids.size());
}

// Verify that GID collection does not include publishers from other nodes,
// even when they publish on the same topic in the same process.
TEST_F(GidFilteringTest, DoesNotCollectExternalPublisherGids)
{
  auto bridge_node = std::make_shared<rclcpp::Node>("bridge_node");
  auto external_node = std::make_shared<rclcpp::Node>("external_node");
  const std::string topic = "/test_gid_external";

  auto bridge_pub = bridge_node->create_publisher<std_msgs::msg::String>(topic, 10);
  auto external_pub = external_node->create_publisher<std_msgs::msg::String>(topic, 10);
  (void)external_pub;

  std::this_thread::sleep_for(100ms);

  // Only bridge_node's publisher should be collected.
  auto self_gids = collect_self_publisher_gids(bridge_node, topic);
  EXPECT_EQ(1u, self_gids.size());

  // But the topic has 2 publishers total.
  auto all_pubs = bridge_node->get_publishers_info_by_topic(topic);
  EXPECT_EQ(2u, all_pubs.size());
}

// A subscriber using GID-based filtering should accept messages from an
// external node while rejecting messages published by its own node.
TEST_F(GidFilteringTest, FiltersOwnMessagesAllowsExternal)
{
  auto bridge_node = std::make_shared<rclcpp::Node>("bridge_node");
  auto external_node = std::make_shared<rclcpp::Node>("external_node");
  const std::string topic = "/test_gid_filter";

  // Simulate the bridge's own ROS publisher (created by GZ-to-ROS handle).
  auto self_pub = bridge_node->create_publisher<std_msgs::msg::String>(topic, 10);

  // Simulate a composed node's publisher.
  auto ext_pub = external_node->create_publisher<std_msgs::msg::String>(topic, 10);

  std::this_thread::sleep_for(100ms);

  // Collect self GIDs.
  auto self_gids = std::make_shared<
    std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>>>(
    collect_self_publisher_gids(bridge_node, topic));
  ASSERT_EQ(1u, self_gids->size());

  // Create subscriber with GID-based filtering.
  std::atomic<int> external_received{0};
  std::atomic<int> self_filtered{0};

  auto sub = bridge_node->create_subscription<std_msgs::msg::String>(
    topic, 10,
    [self_gids, &external_received, &self_filtered](
      const std_msgs::msg::String & /*msg*/,
      const rclcpp::MessageInfo & msg_info)
    {
      const auto & sender_gid = msg_info.get_rmw_message_info().publisher_gid;
      if (is_from_self(sender_gid, *self_gids)) {
        self_filtered++;
        return;
      }
      external_received++;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(external_node);

  std_msgs::msg::String msg;

  // Publish from both sources and spin to process callbacks.
  for (int i = 0; i < 20 && external_received == 0; ++i) {
    msg.data = "from_bridge";
    self_pub->publish(msg);
    msg.data = "from_external";
    ext_pub->publish(msg);
    std::this_thread::sleep_for(50ms);
    executor.spin_some();
  }

  EXPECT_GT(external_received.load(), 0)
    << "Messages from an external composed node must be received";
  EXPECT_GT(self_filtered.load(), 0)
    << "Messages from the bridge's own publisher must be caught by the GID filter";
}
