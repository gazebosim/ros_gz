/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <chrono>
#include "../test_utils.h"

//////////////////////////////////////////////////
/// \brief A class for testing ROS 1 topic subscription.
template <typename ROS_T>
class MyTestClass
{
  /// \brief Class constructor.
  public: MyTestClass(const std::string &_topic)
  {
    this->sub = this->n.subscribe(_topic, 1000, &MyTestClass::Cb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void Cb(const ROS_T& _msg)
  {
    ros1_ign_bridge::testing::compareTestMsg(_msg);
    this->callbackExecuted = true;
  };

  /// \brief Member variables that flag when the actions are executed.
  public: bool callbackExecuted = false;

  /// \brief ROS node handle;
  private: ros::NodeHandle n;

  /// \brief ROS subscriber;
  private: ros::Subscriber sub;
};

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Float)
{
  MyTestClass<std_msgs::Float32> client("float");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Header)
{
  MyTestClass<std_msgs::Header> client("header");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, String)
{
  MyTestClass<std_msgs::String> client("string");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Quaternion)
{
  MyTestClass<geometry_msgs::Quaternion> client("quaternion");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Vector3)
{
  MyTestClass<geometry_msgs::Vector3> client("vector3");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Clock)
{
  MyTestClass<rosgraph_msgs::Clock> client("clock");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Point)
{
  MyTestClass<geometry_msgs::Point> client("point");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Pose)
{
  MyTestClass<geometry_msgs::Pose> client("pose");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, PoseStamped)
{
  MyTestClass<geometry_msgs::PoseStamped> client("pose_stamped");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Transform)
{
  MyTestClass<geometry_msgs::Transform> client("transform");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, TransformStamped)
{
  MyTestClass<geometry_msgs::TransformStamped> client("transform_stamped");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Twist)
{
  MyTestClass<geometry_msgs::Twist> client("twist");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Image)
{
  MyTestClass<sensor_msgs::Image> client("image");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, CameraInfo)
{
  MyTestClass<sensor_msgs::CameraInfo> client("camera_info");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, FluidPressure)
{
  MyTestClass<sensor_msgs::FluidPressure> client("fluid_pressure");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Imu)
{
  MyTestClass<sensor_msgs::Imu> client("imu");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, JointStates)
{
  MyTestClass<sensor_msgs::JointState> client("joint_states");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, LaserScan)
{
  MyTestClass<sensor_msgs::LaserScan> client("laserscan");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, MagneticField)
{
  MyTestClass<sensor_msgs::MagneticField> client("magnetic");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Actuators)
{
  MyTestClass<mav_msgs::Actuators> client("actuators");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Odometry)
{
  MyTestClass<nav_msgs::Odometry> client("odometry");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros1_string_subscriber");

  return RUN_ALL_TESTS();
}
