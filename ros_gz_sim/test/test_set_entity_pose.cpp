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
#include <geometry_msgs/msg/quaternion.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <ros_gz_sim/set_entity_pose.hpp>

using namespace std::chrono_literals;

// Test fixture for the EntityPoseSetter
class EntityPoseSetterTest : public ::testing::Test
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

// Simple implementation of the set entity pose service for testing
class TestSetEntityPoseService : public rclcpp::Node
{
public:
  TestSetEntityPoseService()
  : Node("test_set_entity_pose_service")
  {
    service_ = this->create_service<ros_gz_interfaces::srv::SetEntityPose>(
      "/world/default/set_pose",
      [this](
        const ros_gz_interfaces::srv::SetEntityPose::Request::SharedPtr request,
        ros_gz_interfaces::srv::SetEntityPose::Response::SharedPtr response) {
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
  ros_gz_interfaces::srv::SetEntityPose::Request get_last_request() const
  {
    return last_request_;
  }

private:
  rclcpp::Service<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr service_;
  ros_gz_interfaces::srv::SetEntityPose::Request last_request_;
  ros_gz_interfaces::srv::SetEntityPose::Response response_;
};

// Integration test for the EntityPoseSetter with a mock service
TEST(SetEntityPoseTest, SetEntityPoseIntegration) {
  // Initialize ROS
  rclcpp::init(0, nullptr);

  // Start the test service in a separate thread
  auto test_service = std::make_shared<TestSetEntityPoseService>();
  test_service->set_response(true);  // Set service to return success

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_service);
  std::thread service_thread([&executor]() {
      executor->spin();
    });

  // Create entity pose setter
  auto pose_setter = std::make_shared<EntityPoseSetter>();

  // Give the service time to be discovered
  std::this_thread::sleep_for(1s);

  // Test case 1: Set pose by entity name with quaternion
  {
    std::string entity_name = "test_entity";
    int entity_id = 0;
    int entity_type = 6;  // MODEL type
    double x = 1.0, y = 2.0, z = 3.0;
    double qx = 0.1, qy = 0.2, qz = 0.3, qw = 0.9165;
    bool use_quaternion = true;

    bool result = pose_setter->set_entity_pose(
      entity_name, entity_id, entity_type, x, y, z, qx, qy, qz, qw, use_quaternion);

    // Verify the result and the request received by the service
    EXPECT_TRUE(result);
    auto last_request = test_service->get_last_request();
    EXPECT_EQ(last_request.entity.name, entity_name);
    EXPECT_EQ(last_request.entity.id, 0);  // ID should be 0 when using name
    EXPECT_EQ(last_request.entity.type, entity_type);
    EXPECT_DOUBLE_EQ(last_request.pose.position.x, x);
    EXPECT_DOUBLE_EQ(last_request.pose.position.y, y);
    EXPECT_DOUBLE_EQ(last_request.pose.position.z, z);
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.x, qx);
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.y, qy);
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.z, qz);
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.w, qw);
  }

  // Test case 2: Set pose by entity ID with euler angles
  {
    std::string entity_name = "";
    int entity_id = 42;
    int entity_type = 6;  // MODEL type
    double x = 4.0, y = 5.0, z = 6.0;
    double roll = 0.1, pitch = 0.2, yaw = 0.3;  // Euler angles in radians
    bool use_quaternion = false;

    bool result = pose_setter->set_entity_pose(
      entity_name, entity_id, entity_type, x, y, z, roll, pitch, yaw, 0.0, use_quaternion);

    // Calculate expected quaternion for verification
    tf2::Quaternion expected_quat;
    expected_quat.setRPY(roll, pitch, yaw);

    // Verify the result and the request received by the service
    EXPECT_TRUE(result);
    auto last_request = test_service->get_last_request();
    EXPECT_EQ(last_request.entity.name, "");  // Name should be empty when using ID
    EXPECT_EQ(last_request.entity.id, entity_id);
    EXPECT_EQ(last_request.entity.type, entity_type);
    EXPECT_DOUBLE_EQ(last_request.pose.position.x, x);
    EXPECT_DOUBLE_EQ(last_request.pose.position.y, y);
    EXPECT_DOUBLE_EQ(last_request.pose.position.z, z);
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.x, expected_quat.x());
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.y, expected_quat.y());
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.z, expected_quat.z());
    EXPECT_DOUBLE_EQ(last_request.pose.orientation.w, expected_quat.w());
  }

  // Test case 3: Test failure response
  {
    test_service->set_response(false);
    std::string entity_name = "test_entity";
    bool result = pose_setter->set_entity_pose(
      entity_name, 0, 6, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);
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
TEST(SetEntityPoseTest, InvalidInputs) {
  // Initialize ROS
  rclcpp::init(0, nullptr);

  // Start the test service
  auto test_service = std::make_shared<TestSetEntityPoseService>();
  test_service->set_response(true);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_service);
  std::thread service_thread([&executor]() {
      executor->spin();
    });

  // Create entity pose setter
  auto pose_setter = std::make_shared<EntityPoseSetter>();

  // Give the service time to be discovered
  std::this_thread::sleep_for(1s);

  // Test case: Missing both entity name and ID
  bool result = pose_setter->set_entity_pose(
    "", 0, 6, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);
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
