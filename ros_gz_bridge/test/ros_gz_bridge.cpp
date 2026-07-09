#include <gtest/gtest.h>

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>

#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

size_t g_service_bridge_create_logs = 0;

class TestableRosGzBridge : public ros_gz_bridge::RosGzBridge
{
public:
  explicit TestableRosGzBridge(const rclcpp::NodeOptions & options)
  : ros_gz_bridge::RosGzBridge(options)
  {
  }

  using ros_gz_bridge::RosGzBridge::spin;

  size_t HandleCount() const
  {
    return handles_.size();
  }

  size_t ServiceCount() const
  {
    return services_.size();
  }
};

class RosGzBridgeTest : public ::testing::Test
{
protected:
  rcutils_logging_output_handler_t previous_output_handler;

  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    g_service_bridge_create_logs = 0;
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);

    auto logging_handler = [](
      const rcutils_log_location_t *,
      int, const char *, rcutils_time_point_value_t,
      const char * format, va_list * args) -> void
      {
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);

        const std::string message = buffer;
        if (message.find("Creating ROS->GZ service bridge") != std::string::npos) {
          ++g_service_bridge_create_logs;
        }
      };

    this->previous_output_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(logging_handler);
  }

  void TearDown() override
  {
    rcutils_logging_set_output_handler(this->previous_output_handler);
  }
};

TEST_F(RosGzBridgeTest, ServiceOnlyConfigIsLoadedOnce)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("config_file", "test/config/service_only.yaml");

  auto bridge = std::make_shared<TestableRosGzBridge>(options);

  bridge->spin();
  EXPECT_EQ(0u, bridge->HandleCount());
  EXPECT_EQ(1u, bridge->ServiceCount());
  EXPECT_EQ(1u, g_service_bridge_create_logs);

  bridge->spin();
  EXPECT_EQ(0u, bridge->HandleCount());
  EXPECT_EQ(1u, bridge->ServiceCount());
  EXPECT_EQ(1u, g_service_bridge_create_logs);
}
