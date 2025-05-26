// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include "ros_subscriber.hpp"
#include "utils/test_utils.hpp"
#include "utils/ros_test_msg.hpp"

#include "ros_gz_interfaces/srv/control_world.hpp"

static std::shared_ptr<rclcpp::Node> kTestNode;

using ros_subscriber::MyTestClass;

/////////////////////////////////////////////////
rclcpp::Node * ros_subscriber::TestNode()
{
  if (kTestNode == nullptr) {
    kTestNode = rclcpp::Node::make_shared("ros_subscriber");
  }
  return kTestNode.get();
}

/////////////////////////////////////////////////
TEST(LaunchActionSubscriberTest, boolean_std_msgs_bool)
{
  MyTestClass<std_msgs::msg::Bool> client("boolean_std_msgs_bool");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(LaunchActionSubscriberTest, imu_sensors_msgs_imu)
{
  MyTestClass<sensor_msgs::msg::Imu> client("imu_sensor_msgs_imu");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(LaunchActionSubscriberTest, WorldControl)
{
  using namespace std::chrono_literals;
  auto node = ros_subscriber::TestNode();
  auto client = node->create_client<ros_gz_interfaces::srv::ControlWorld>(
    "/gz_ros/test/serviceclient/world_control");
  std::this_thread::sleep_for(1s);
  ASSERT_TRUE(client->wait_for_service(5s));
  const auto msg = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
  auto future = client->async_send_request(msg);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(kTestNode);
  ex.spin_until_future_complete(future);
  const auto res = future.get();
  ASSERT_TRUE(res->success);
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto ret = RUN_ALL_TESTS();
  kTestNode.reset();
  return ret;
}
