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

#include <memory>
#include <string>
#include <chrono>
#include <future>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>

#include <ros_gz_sim/spawn_entity.hpp>

using namespace std::chrono_literals;

// Test fixture for command line arguments parsing
TEST(SpawnEntityTest, ArgumentParsing) {
  const char * argv[] = {
    "spawn_entity",
    "--name", "test_model",
    "--sdf_filename", "model.sdf",
    "--pos", "1.0", "2.0", "3.0",
    "--euler", "0.1", "0.2", "0.3"
  };
  int argc = sizeof(argv) / sizeof(argv[0]);

  CommandLineArgs args = parse_arguments(argc, const_cast<char **>(argv));

  EXPECT_EQ(args.model_name, "test_model");
  EXPECT_EQ(args.sdf_filename, "model.sdf");
  EXPECT_EQ(args.position.size(), 3u);
  EXPECT_DOUBLE_EQ(args.position[0], 1.0);
  EXPECT_DOUBLE_EQ(args.position[1], 2.0);
  EXPECT_DOUBLE_EQ(args.position[2], 3.0);
  EXPECT_EQ(args.euler.size(), 3u);
  EXPECT_DOUBLE_EQ(args.euler[0], 0.1);
  EXPECT_DOUBLE_EQ(args.euler[1], 0.2);
  EXPECT_DOUBLE_EQ(args.euler[2], 0.3);
}

// Test with quaternion arguments
TEST(SpawnEntityTest, QuaternionArgumentParsing) {
  const char * argv[] = {
    "spawn_entity",
    "--name", "test_model",
    "--sdf_filename", "model.sdf",
    "--pos", "1.0", "2.0", "3.0",
    "--quat", "0.1", "0.2", "0.3", "0.9"
  };
  int argc = sizeof(argv) / sizeof(argv[0]);

  CommandLineArgs args = parse_arguments(argc, const_cast<char **>(argv));

  EXPECT_EQ(args.model_name, "test_model");
  EXPECT_EQ(args.sdf_filename, "model.sdf");
  EXPECT_EQ(args.position.size(), 3u);
  EXPECT_DOUBLE_EQ(args.position[0], 1.0);
  EXPECT_DOUBLE_EQ(args.position[1], 2.0);
  EXPECT_DOUBLE_EQ(args.position[2], 3.0);
  EXPECT_EQ(args.quaternion.size(), 4u);
  EXPECT_DOUBLE_EQ(args.quaternion[0], 0.1);
  EXPECT_DOUBLE_EQ(args.quaternion[1], 0.2);
  EXPECT_DOUBLE_EQ(args.quaternion[2], 0.3);
  EXPECT_DOUBLE_EQ(args.quaternion[3], 0.9);
}

// Simple implementation of the spawn service for testing
class TestSpawnService : public rclcpp::Node
{
public:
  TestSpawnService()
  : Node("test_spawn_service")
  {
    service_ = this->create_service<ros_gz_interfaces::srv::SpawnEntity>(
      "/world/default/create",
      [this](
        const ros_gz_interfaces::srv::SpawnEntity::Request::SharedPtr request,
        ros_gz_interfaces::srv::SpawnEntity::Response::SharedPtr response) {
        last_request_ = *request;
        *response = response_;
        });
  }

  // Set the response that will be returned by the service
  void set_response(bool success)
  {
    response_.success = success;
  }

  // Get the last request received by the service
  ros_gz_interfaces::srv::SpawnEntity::Request get_last_request() const
  {
    return last_request_;
  }

private:
  rclcpp::Service<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr service_;
  ros_gz_interfaces::srv::SpawnEntity::Request last_request_;
  ros_gz_interfaces::srv::SpawnEntity::Response response_;
};

// Integration test for the EntitySpawner with a simple test service
TEST(SpawnEntityTest, SpawnEntityIntegration) {
  // Initialize ROS
  rclcpp::init(0, nullptr);

  // Start the test service in a separate thread
  auto test_service = std::make_shared<TestSpawnService>();
  test_service->set_response(true);  // Set service to return success

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_service);
  std::thread service_thread([&executor]() {
      executor->spin();
    });

  // Create entity spawner
  auto spawner = std::make_shared<EntitySpawner>();

  // Give the service time to be discovered
  std::this_thread::sleep_for(1s);

  // Test parameters
  std::string model_name = "test_model";
  std::string sdf_filename = "test_model.sdf";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.w = 1.0;

  // Test spawning entity
  bool result = spawner->spawn_entity(model_name, sdf_filename, pose);

  // Verify the result and the request received by the service
  EXPECT_TRUE(result);
  auto last_request = test_service->get_last_request();
  EXPECT_EQ(last_request.entity_factory.name, model_name);
  EXPECT_EQ(last_request.entity_factory.sdf_filename, sdf_filename);
  EXPECT_DOUBLE_EQ(last_request.entity_factory.pose.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(last_request.entity_factory.pose.position.y, pose.position.y);
  EXPECT_DOUBLE_EQ(last_request.entity_factory.pose.position.z, pose.position.z);
  EXPECT_DOUBLE_EQ(last_request.entity_factory.pose.orientation.w, pose.orientation.w);

  // Test failure case
  test_service->set_response(false);
  result = spawner->spawn_entity(model_name, sdf_filename, pose);
  EXPECT_FALSE(result);

  // Clean up
  executor->cancel();
  if (service_thread.joinable()) {
    service_thread.join();
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
