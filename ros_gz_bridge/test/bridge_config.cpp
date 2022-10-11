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

#include <ros_gz_bridge/bridge_config.hpp>

TEST(BridgeConfig, Minimum)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/minimum.yaml");
  EXPECT_EQ(4u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("chatter", config.ros_topic_name);
    EXPECT_EQ("chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[1];
    EXPECT_EQ("chatter_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_ros", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[2];
    EXPECT_EQ("chatter_gz", config.ros_topic_name);
    EXPECT_EQ("chatter_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[3];
    EXPECT_EQ("chatter_both_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_both_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
}

TEST(BridgeConfig, MinimumIgn)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/minimum_ign.yaml");
  EXPECT_EQ(4u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("chatter", config.ros_topic_name);
    EXPECT_EQ("chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[1];
    EXPECT_EQ("chatter_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_ros", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[2];
    EXPECT_EQ("chatter_gz", config.ros_topic_name);
    EXPECT_EQ("chatter_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[3];
    EXPECT_EQ("chatter_both_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_both_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
}


TEST(BridgeConfig, FullGz)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/full.yaml");
  EXPECT_EQ(2u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(6u, config.publisher_queue_size);
    EXPECT_EQ(5u, config.subscriber_queue_size);
    EXPECT_EQ(true, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, config.direction);
  }

  {
    auto config = results[1];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(20u, config.publisher_queue_size);
    EXPECT_EQ(10u, config.subscriber_queue_size);
    EXPECT_EQ(false, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, config.direction);
  }
}

TEST(BridgeConfig, FullIgn)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/full.yaml");
  EXPECT_EQ(2u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(6u, config.publisher_queue_size);
    EXPECT_EQ(5u, config.subscriber_queue_size);
    EXPECT_EQ(true, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, config.direction);
  }

  {
    auto config = results[1];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(20u, config.publisher_queue_size);
    EXPECT_EQ(10u, config.subscriber_queue_size);
    EXPECT_EQ(false, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, config.direction);
  }
}

TEST(BridgeConfig, InvalidSetTwoRos)
{
  // Cannot set topic_name and ros_topic_name
  auto yaml = R"(
- topic_name: foo
  ros_topic_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
}

TEST(BridgeConfig, InvalidSetTwoGz)
{
  // Cannot set topic_name and gz_topic_name
  auto yaml = R"(
- topic_name: foo
  gz_topic_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
}

TEST(BridgeConfig, InvalidSetTypes)
{
  // Both ros_type_name and gz_type_name must be set
  auto yaml = R"(
- topic_name: foo
  ros_type_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
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
    gz_type_name: ignition.msgs.StringMsg)";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::BIDIRECTIONAL, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: BIDIRECTIONAL
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::BIDIRECTIONAL, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: ROS_TO_GZ
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: GZ_TO_ROS
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, results[0].direction);
  }


  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: asdfasdfasdfasdf
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(0u, results.size());
  }
}
