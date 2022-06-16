// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <ros_ign_bridge/bridge_config.hpp>

TEST(BridgeConfig, minimum)
{
  auto results = ros_ign_bridge::readFromYamlFile("test/config/minimum.yaml");
  EXPECT_EQ(4u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("chatter", config.ros_topic_name);
    EXPECT_EQ("chatter", config.ign_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.ign_type_name);
    EXPECT_EQ(ros_ign_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[1];
    EXPECT_EQ("chatter_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_ros", config.ign_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.ign_type_name);
    EXPECT_EQ(ros_ign_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[2];
    EXPECT_EQ("chatter_ign", config.ros_topic_name);
    EXPECT_EQ("chatter_ign", config.ign_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.ign_type_name);
    EXPECT_EQ(ros_ign_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[3];
    EXPECT_EQ("chatter_both_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_both_ign", config.ign_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.ign_type_name);
    EXPECT_EQ(ros_ign_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_ign_bridge::kDefaultLazy, config.is_lazy);
  }
}

TEST(BridgeConfig, full)
{
  auto results = ros_ign_bridge::readFromYamlFile("test/config/full.yaml");
  EXPECT_EQ(1u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("ign_chatter", config.ign_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.ign_type_name);
    EXPECT_EQ(6u, config.publisher_queue_size);
    EXPECT_EQ(5u, config.subscriber_queue_size);
    EXPECT_EQ(true, config.is_lazy);
  }
}

TEST(BridgeConfig, InvalidSetTwoRos)
{
  // Cannot set topic_name and ros_topic_name
  auto yaml = R"(
- topic_name: foo
  ros_topic_name: bar)";

  auto results = ros_ign_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
}

TEST(BridgeConfig, InvalidSetTwoIgn)
{
  // Cannot set topic_name and ign_topic_name
  auto yaml = R"(
- topic_name: foo
  ign_topic_name: bar)";

  auto results = ros_ign_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
}

TEST(BridgeConfig, InvalidSetTypes)
{
  // Both ros_type_name and ign_type_name must be set
  auto yaml = R"(
- topic_name: foo
  ros_type_name: bar)";

  auto results = ros_ign_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
}

TEST(BridgeConfig, ParseDirection)
{
  {
    // Check that default is bidirectional
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    ign_type_name: ignition.msgs.StringMsg)";

    auto results = ros_ign_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_ign_bridge::BridgeDirection::BIDIRECTIONAL, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    ign_type_name: ignition.msgs.StringMsg
    direction: BIDIRECTIONAL
    )";

    auto results = ros_ign_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_ign_bridge::BridgeDirection::BIDIRECTIONAL, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    ign_type_name: ignition.msgs.StringMsg
    direction: ROS_TO_IGN
    )";

    auto results = ros_ign_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_ign_bridge::BridgeDirection::ROS_TO_IGN, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    ign_type_name: ignition.msgs.StringMsg
    direction: IGN_TO_ROS
    )";

    auto results = ros_ign_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_ign_bridge::BridgeDirection::IGN_TO_ROS, results[0].direction);
  }


  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    ign_type_name: ignition.msgs.StringMsg
    direction: asdfasdfasdfasdf
    )";

    auto results = ros_ign_bridge::readFromYamlString(yaml);
    EXPECT_EQ(0u, results.size());
  }
}
