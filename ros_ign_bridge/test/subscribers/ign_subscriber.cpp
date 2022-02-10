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
#include <ignition/transport.hh>

#include <chrono>
#include <memory>
#include <string>

#include "utils/test_utils.hpp"
#include "utils/ign_test_msg.hpp"

//////////////////////////////////////////////////
/// \brief A class for testing Ignition Transport topic subscription.
template<typename IGN_T>
class MyTestClass
{
  /// \brief Class constructor.
  /// \param[in] _topic Topic to subscribe.

public:
  explicit MyTestClass(const std::string & _topic)
  {
    EXPECT_TRUE(this->node.Subscribe(_topic, &MyTestClass::Cb, this));
  }

  /// \brief Member function called each time a topic update is received.

public:
  void Cb(const IGN_T & _msg)
  {
    ros_ign_bridge::testing::compareTestMsg(std::make_shared<IGN_T>(_msg));
    this->callbackExecuted = true;
  }

  /// \brief Member variables that flag when the actions are executed.

public:
  bool callbackExecuted = false;

  /// \brief Transport node;

private:
  ignition::transport::Node node;
};

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Boolean)
{
  MyTestClass<ignition::msgs::Boolean> client("bool");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Empty)
{
  MyTestClass<ignition::msgs::Empty> client("empty");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Float)
{
  MyTestClass<ignition::msgs::Float> client("float");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Double)
{
  MyTestClass<ignition::msgs::Double> client("double");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, UInt32)
{
  MyTestClass<ignition::msgs::UInt32> client("uint32");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Header)
{
  MyTestClass<ignition::msgs::Header> client("header");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, String)
{
  MyTestClass<ignition::msgs::StringMsg> client("string");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Quaternion)
{
  MyTestClass<ignition::msgs::Quaternion> client("quaternion");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Vector3)
{
  MyTestClass<ignition::msgs::Vector3d> client("vector3");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Clock)
{
  MyTestClass<ignition::msgs::Clock> client("clock");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Point)
{
  MyTestClass<ignition::msgs::Vector3d> client("point");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Pose)
{
  MyTestClass<ignition::msgs::Pose> client("pose");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, PoseStamped)
{
  MyTestClass<ignition::msgs::Pose> client("pose_stamped");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Transform)
{
  MyTestClass<ignition::msgs::Pose> client("transform");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, TransformStamped)
{
  MyTestClass<ignition::msgs::Pose> client("transform_stamped");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, TF2Message)
{
  MyTestClass<ignition::msgs::Pose_V> client("tf2_message");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Twist)
{
  MyTestClass<ignition::msgs::Twist> client("twist");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Wrench)
{
  MyTestClass<ignition::msgs::Wrench> client("wrench");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, JointWrench)
{
  MyTestClass<ignition::msgs::JointWrench> client("joint_wrench");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Entity)
{
  MyTestClass<ignition::msgs::Entity> client("entity");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Contact)
{
  MyTestClass<ignition::msgs::Contact> client("contact");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Contacts)
{
  MyTestClass<ignition::msgs::Contacts> client("contacts");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Image)
{
  MyTestClass<ignition::msgs::Image> client("image");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 500);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, CameraInfo)
{
  MyTestClass<ignition::msgs::CameraInfo> client("camera_info");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, FluidPressure)
{
  MyTestClass<ignition::msgs::FluidPressure> client("fluid_pressure");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Imu)
{
  MyTestClass<ignition::msgs::IMU> client("imu");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, LaserScan)
{
  MyTestClass<ignition::msgs::LaserScan> client("laserscan");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Magnetometer)
{
  MyTestClass<ignition::msgs::Magnetometer> client("magnetic");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

// /////////////////////////////////////////////////
// TEST(IgnSubscriberTest, Actuators)
// {
//   MyTestClass<ignition::msgs::Actuators> client("actuators");
//
//   using namespace std::chrono_literals;
//   ros_ign_bridge::testing::waitUntilBoolVar(
//     client.callbackExecuted, 100ms, 200);
//
//   EXPECT_TRUE(client.callbackExecuted);
// }

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Odometry)
{
  MyTestClass<ignition::msgs::Odometry> client("odometry");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, JointStates)
{
  MyTestClass<ignition::msgs::Model> client("joint_states");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, PointCloudPacked)
{
  MyTestClass<ignition::msgs::PointCloudPacked> client("pointcloud2");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, BatteryState)
{
  MyTestClass<ignition::msgs::BatteryState> client("battery_state");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, GuiCamera)
{
  MyTestClass<ignition::msgs::GUICamera> client("gui_camera");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, JointTrajectory)
{
  MyTestClass<ignition::msgs::JointTrajectory> client("joint_trajectory");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, StringMsg_V)
{
  MyTestClass<ignition::msgs::StringMsg_V> client("stringmsg_v");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Time)
{
  MyTestClass<ignition::msgs::Time> client("time");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, TrackVisual)
{
  MyTestClass<ignition::msgs::TrackVisual> client("track_visual");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, VideoRecord)
{
  MyTestClass<ignition::msgs::VideoRecord> client("video_record");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
