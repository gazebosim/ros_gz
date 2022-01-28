// Copyright 2021 Open Source Robotics Foundation, Inc.
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
//

#include <gtest/gtest.h>

#include <chrono>

#include "ros_subscriber.hpp"

using ros_subscriber::MyTestClass;

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Quaternion)
{
  MyTestClass<geometry_msgs::msg::Quaternion> client("quaternion");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Vector3)
{
  MyTestClass<geometry_msgs::msg::Vector3> client("vector3");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Point)
{
  MyTestClass<geometry_msgs::msg::Point> client("point");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Pose)
{
  MyTestClass<geometry_msgs::msg::Pose> client("pose");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, PoseStamped)
{
  MyTestClass<geometry_msgs::msg::PoseStamped> client("pose_stamped");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Transform)
{
  MyTestClass<geometry_msgs::msg::Transform> client("transform");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, TransformStamped)
{
  MyTestClass<geometry_msgs::msg::TransformStamped> client("transform_stamped");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, TF2Message)
{
  MyTestClass<tf2_msgs::msg::TFMessage> client("tf2_message");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Twist)
{
  MyTestClass<geometry_msgs::msg::Twist> client("twist");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Wrench)
{
  MyTestClass<geometry_msgs::msg::Wrench> client("wrench");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}
