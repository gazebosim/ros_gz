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
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>

#include <ros_gz_sim/delete_entity.hpp>

using namespace std::chrono_literals;

// Test fixture for the EntityDeleter
class EntityDeleterTest : public ::testing::Test
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
};

// Simple implementation of the delete entity service for testing
class TestDeleteEntityService : public rclcpp::Node
{
public:
  TestDeleteEntityService()
  : Node("test_delete_entity_service")
  {
    service_ = this->create_service<ros_gz_interfaces::srv::DeleteEntity>(
      "/world/default/remove",
      [this](
        const ros_gz_interfaces::srv::DeleteEntity::Request::SharedPtr request,
        ros_gz_interfaces::srv::DeleteEntity::Response::SharedPtr response) {
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
  ros_gz_interfaces::srv::DeleteEntity::Request get_last_request() const
  {
    return last_request_;
  }

private:
  rclcpp::Service<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr service_;
  ros_gz_interfaces::srv::DeleteEntity::Request last_request_;
  ros_gz_interfaces::srv::DeleteEntity::Response response_;
};

// Integration test for the EntityDeleter with a mock service
TEST(DeleteEntityTest, DeleteEntityIntegration) {
  // Initialize ROS
  rclcpp::init(0, nullptr);

  // Start the test service in a separate thread
  auto test_service = std::make_shared<TestDeleteEntityService>();
  test_service->set_response(true);  // Set service to return success

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_service);
  std::thread service_thread([&executor]() {
      executor->spin();
    });

  // Create entity deleter
  auto deleter = std::make_shared<EntityDeleter>();

  // Give the service time to be discovered
  std::this_thread::sleep_for(1s);

  // Test case 1: Delete entity by name
  {
    std::string entity_name = "test_entity";
    int entity_id = 0;
    int entity_type = 6;  // MODEL type

    bool result = deleter->delete_entity(entity_name, entity_id, entity_type);

    // Verify the result and the request received by the service
    EXPECT_TRUE(result);
    auto last_request = test_service->get_last_request();
    EXPECT_EQ(last_request.entity.name, entity_name);
    EXPECT_EQ(last_request.entity.id, 0);  // ID should be 0 when using name
    EXPECT_EQ(last_request.entity.type, entity_type);
  }

  // Test case 2: Delete entity by ID
  {
    std::string entity_name = "";
    int entity_id = 42;
    int entity_type = 6;  // MODEL type

    bool result = deleter->delete_entity(entity_name, entity_id, entity_type);

    // Verify the result and the request received by the service
    EXPECT_TRUE(result);
    auto last_request = test_service->get_last_request();
    EXPECT_EQ(last_request.entity.name, "");  // Name should be empty when using ID
    EXPECT_EQ(last_request.entity.id, entity_id);
    EXPECT_EQ(last_request.entity.type, entity_type);
  }

  // Test case 3: Delete with different entity type
  {
    std::string entity_name = "test_light";
    int entity_id = 0;
    int entity_type = 1;  // LIGHT type

    bool result = deleter->delete_entity(entity_name, entity_id, entity_type);

    // Verify the result and the request received by the service
    EXPECT_TRUE(result);
    auto last_request = test_service->get_last_request();
    EXPECT_EQ(last_request.entity.name, entity_name);
    EXPECT_EQ(last_request.entity.type, entity_type);
  }

  // Test case 4: Test failure response
  {
    test_service->set_response(false);
    std::string entity_name = "nonexistent_entity";
    bool result = deleter->delete_entity(entity_name, 0, 6);
    EXPECT_FALSE(result);
  }

  // Clean up
  executor->cancel();
  if (service_thread.joinable()) {
    service_thread.join();
  }
  rclcpp::shutdown();
}

// Test for invalid inputs
TEST(DeleteEntityTest, InvalidInputs) {
  // Initialize ROS
  rclcpp::init(0, nullptr);

  // Start the test service
  auto test_service = std::make_shared<TestDeleteEntityService>();
  test_service->set_response(true);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_service);
  std::thread service_thread([&executor]() {
      executor->spin();
    });

  // Create entity deleter
  auto deleter = std::make_shared<EntityDeleter>();

  // Give the service time to be discovered
  std::this_thread::sleep_for(1s);

  // Test case: Missing both entity name and ID
  bool result = deleter->delete_entity("", 0, 6);
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
