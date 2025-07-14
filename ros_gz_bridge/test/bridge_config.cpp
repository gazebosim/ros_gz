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

#include "rcutils/logging.h"

size_t g_log_calls = 0;

struct LogEvent
{
  const rcutils_log_location_t * location;
  int level;
  std::string name;
  rcutils_time_point_value_t timestamp;
  std::string message;
};
LogEvent g_last_log_event;

class BridgeConfig : public ::testing::Test
{
public:
  rcutils_logging_output_handler_t previous_output_handler;
  void SetUp()
  {
    g_log_calls = 0;
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

    auto rcutils_logging_console_output_handler = [](
      const rcutils_log_location_t * location,
      int level, const char * name, rcutils_time_point_value_t timestamp,
      const char * format, va_list * args) -> void
      {
        g_log_calls += 1;
        g_last_log_event.location = location;
        g_last_log_event.level = level;
        g_last_log_event.name = name ? name : "";
        g_last_log_event.timestamp = timestamp;
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);
        g_last_log_event.message = buffer;
      };

    this->previous_output_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
  }

  void TearDown()
  {
    rcutils_logging_set_output_handler(this->previous_output_handler);
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
    EXPECT_FALSE(g_rcutils_logging_initialized);
  }
};


TEST_F(BridgeConfig, Minimum)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/minimum.yaml");
  EXPECT_EQ(5u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("chatter", config.ros_topic_name);
    EXPECT_EQ("chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_FALSE(config.publisher_queue_size.has_value());
    EXPECT_FALSE(config.subscriber_queue_size.has_value());
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
    EXPECT_FALSE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.PublisherQoS());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.SubscriberQoS());
  }
  {
    auto config = results[1];
    EXPECT_EQ("chatter_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_ros", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_FALSE(config.publisher_queue_size.has_value());
    EXPECT_FALSE(config.subscriber_queue_size.has_value());
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
    EXPECT_FALSE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.PublisherQoS());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.SubscriberQoS());
  }
  {
    auto config = results[2];
    EXPECT_EQ("chatter_gz", config.ros_topic_name);
    EXPECT_EQ("chatter_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_FALSE(config.publisher_queue_size.has_value());
    EXPECT_FALSE(config.subscriber_queue_size.has_value());
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
    EXPECT_FALSE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.PublisherQoS());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.SubscriberQoS());
  }
  {
    auto config = results[3];
    EXPECT_EQ("chatter_both_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_both_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_FALSE(config.publisher_queue_size.has_value());
    EXPECT_FALSE(config.subscriber_queue_size.has_value());
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
    EXPECT_FALSE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.PublisherQoS());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.SubscriberQoS());
  }
  {
    auto config = results[4];
    EXPECT_EQ("/gz_ros/test/serviceclient/world_control", config.service_name);
    EXPECT_EQ("ros_gz_interfaces/srv/ControlWorld", config.ros_type_name);
    EXPECT_EQ("gz.msgs.WorldControl", config.gz_req_type_name);
    EXPECT_EQ("gz.msgs.Boolean", config.gz_rep_type_name);
  }
}

TEST_F(BridgeConfig, FullGz)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/full.yaml");
  EXPECT_EQ(3u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(6u, config.publisher_queue_size.value_or(0));
    EXPECT_EQ(5u, config.subscriber_queue_size.value_or(0));
    EXPECT_EQ(true, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, config.direction);
    EXPECT_FALSE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(6u)), config.PublisherQoS());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(5u)), config.SubscriberQoS());
  }

  {
    auto config = results[1];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(20u, config.publisher_queue_size.value_or(0));
    EXPECT_EQ(10u, config.subscriber_queue_size.value_or(0));
    EXPECT_EQ(false, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, config.direction);
    EXPECT_FALSE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(20u)), config.PublisherQoS());
    EXPECT_EQ(rclcpp::QoS(rclcpp::KeepLast(10u)), config.SubscriberQoS());
  }

  {
    auto config = results[2];
    EXPECT_EQ("/gz_ros/test/serviceclient/world_control", config.service_name);
    EXPECT_EQ("ros_gz_interfaces/srv/ControlWorld", config.ros_type_name);
    EXPECT_EQ("gz.msgs.WorldControl", config.gz_req_type_name);
    EXPECT_EQ("gz.msgs.Boolean", config.gz_rep_type_name);
  }
}

TEST_F(BridgeConfig, QoSFullGz)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/qos.yaml");
  EXPECT_EQ(2u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_FALSE(config.publisher_queue_size.has_value());
    EXPECT_FALSE(config.subscriber_queue_size.has_value());
    EXPECT_EQ(true, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, config.direction);
    ASSERT_TRUE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::SensorDataQoS(), *config.qos_profile);
    EXPECT_EQ(rclcpp::SensorDataQoS(), config.PublisherQoS());
    EXPECT_EQ(rclcpp::SensorDataQoS(), config.SubscriberQoS());
  }

  {
    auto config = results[1];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(20u, config.publisher_queue_size.value_or(0));
    EXPECT_FALSE(config.subscriber_queue_size.has_value());
    EXPECT_EQ(false, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, config.direction);
    ASSERT_TRUE(config.qos_profile.has_value());
    EXPECT_EQ(rclcpp::ClockQoS(), *config.qos_profile);
    EXPECT_EQ(rclcpp::ClockQoS().keep_last(20u), config.PublisherQoS());
    EXPECT_EQ(rclcpp::ClockQoS(), config.SubscriberQoS());
  }
}

TEST_F(BridgeConfig, InvalidSetTwoRos)
{
  // Cannot set topic_name and ros_topic_name
  auto yaml = R"(
- topic_name: foo
  ros_topic_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse entry: topic_name and ros_topic_name are mutually exclusive",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidSetTwoGz)
{
  // Cannot set topic_name and gz_topic_name
  auto yaml = R"(
- topic_name: foo
  gz_topic_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse entry: topic_name and gz_topic_name are mutually exclusive",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidSetTypes)
{
  // Both ros_type_name and gz_type_name must be set
  auto yaml = R"(
- topic_name: foo
  ros_type_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse entry: both ros_type_name and gz_type_name must be set",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, ParseDirection)
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
    direction: foobar
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(0u, results.size());
    EXPECT_EQ("Could not parse entry: invalid direction [foobar]", g_last_log_event.message);
  }
}

TEST_F(BridgeConfig, InvalidFileDoesntExist)
{
  auto results = ros_gz_bridge::readFromYamlFile("this/should/never/be/a/file.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse config: failed to open file [this/should/never/be/a/file.yaml]",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidTopLevel)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/invalid.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse config: top level must be a YAML sequence",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidQoS)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/invalid_qos.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse entry: Invalid QoS profile 'UNKNOWN'",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, EmptyYAML)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/empty.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse config: file empty [test/config/empty.yaml]",
    g_last_log_event.message);
}
