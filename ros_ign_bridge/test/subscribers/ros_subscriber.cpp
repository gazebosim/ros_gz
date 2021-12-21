// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_ign_interfaces/msg/light.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "../test_utils.hpp"

using std::placeholders::_1;

std::shared_ptr<rclcpp::Node> node;

//////////////////////////////////////////////////
/// \brief A class for testing ROS topic subscription.
template<typename ROS_T>
class MyTestClass
{
  /// \brief Class constructor.

public:
  explicit MyTestClass(const std::string & _topic)
  {
    this->sub =
      node->create_subscription<ROS_T>(_topic, 1000, std::bind(&MyTestClass::Cb, this, _1));
  }

  /// \brief Member function called each time a topic update is received.

public:
  void Cb(const ROS_T & _msg)
  {
    ros_ign_bridge::testing::compareTestMsg(std::make_shared<ROS_T>(_msg));
    this->callbackExecuted = true;
  }

  /// \brief Member variables that flag when the actions are executed.

public:
  bool callbackExecuted = false;

/// \brief ROS subscriber;

private:
  typename rclcpp::Subscription<ROS_T>::SharedPtr sub;
};

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Bool)
{
  MyTestClass<std_msgs::msg::ColorRGBA> client("color");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Bool)
{
  MyTestClass<std_msgs::msg::Bool> client("bool");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Empty)
{
  MyTestClass<std_msgs::msg::Empty> client("empty");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Float)
{
  MyTestClass<std_msgs::msg::Float32> client("float");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Double)
{
  MyTestClass<std_msgs::msg::Float64> client("double");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, UInt32)
{
  MyTestClass<std_msgs::msg::UInt32> client("uint32");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Header)
{
  MyTestClass<std_msgs::msg::Header> client("header");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, String)
{
  MyTestClass<std_msgs::msg::String> client("string");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Quaternion)
{
  MyTestClass<geometry_msgs::msg::Quaternion> client("quaternion");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Vector3)
{
  MyTestClass<geometry_msgs::msg::Vector3> client("vector3");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Clock)
{
  MyTestClass<rosgraph_msgs::msg::Clock> client("clock");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Point)
{
  MyTestClass<geometry_msgs::msg::Point> client("point");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Pose)
{
  MyTestClass<geometry_msgs::msg::Pose> client("pose");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, PoseStamped)
{
  MyTestClass<geometry_msgs::msg::PoseStamped> client("pose_stamped");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Transform)
{
  MyTestClass<geometry_msgs::msg::Transform> client("transform");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, TransformStamped)
{
  MyTestClass<geometry_msgs::msg::TransformStamped> client("transform_stamped");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, TF2Message)
{
  MyTestClass<tf2_msgs::msg::TFMessage> client("tf2_message");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Twist)
{
  MyTestClass<geometry_msgs::msg::Twist> client("twist");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Wrench)
{
  MyTestClass<geometry_msgs::msg::Wrench> client("wrench");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Light)
{
  MyTestClass<ros_ign_interfaces::msg::Light> client("light");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, JointWrench)
{
  MyTestClass<ros_ign_interfaces::msg::JointWrench> client("joint_wrench");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Entity)
{
  MyTestClass<ros_ign_interfaces::msg::Entity> client("entity");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Contact)
{
  MyTestClass<ros_ign_interfaces::msg::Contact> client("contact");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Contacts)
{
  MyTestClass<ros_ign_interfaces::msg::Contacts> client("contacts");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Image)
{
  MyTestClass<sensor_msgs::msg::Image> client("image");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, CameraInfo)
{
  MyTestClass<sensor_msgs::msg::CameraInfo> client("camera_info");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, FluidPressure)
{
  MyTestClass<sensor_msgs::msg::FluidPressure> client("fluid_pressure");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Imu)
{
  MyTestClass<sensor_msgs::msg::Imu> client("imu");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, JointStates)
{
  MyTestClass<sensor_msgs::msg::JointState> client("joint_states");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, LaserScan)
{
  MyTestClass<sensor_msgs::msg::LaserScan> client("laserscan");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, MagneticField)
{
  MyTestClass<sensor_msgs::msg::MagneticField> client("magnetic");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

// /////////////////////////////////////////////////
// TEST(ROSSubscriberTest, Actuators)
// {
//   MyTestClass<mav_msgs::msg::Actuators> client("actuators");
//
//   using namespace std::chrono_literals;
//   ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
//     node, client.callbackExecuted, 10ms, 200);
//
//   EXPECT_TRUE(client.callbackExecuted);
// }

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, Odometry)
{
  MyTestClass<nav_msgs::msg::Odometry> client("odometry");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, PointCloud2)
{
  MyTestClass<sensor_msgs::msg::PointCloud2> client("pointcloud2");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, BatteryState)
{
  MyTestClass<sensor_msgs::msg::BatteryState> client("battery_state");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, JointTrajectory)
{
  MyTestClass<trajectory_msgs::msg::JointTrajectory> client("joint_trajectory");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVarAndSpin(
    node, client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ros_string_subscriber");

  return RUN_ALL_TESTS();
}
