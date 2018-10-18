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
#include <chrono>
#include <string>
#include <ignition/transport.hh>
#include "../test_config.h"

//////////////////////////////////////////////////
/// \brief A class for testing Ignition Transport topic subscription.
template <typename IGN_T>
class MyTestClass
{
  /// \brief Class constructor.
  /// \param[in] _topic Topic to subscribe.
  public: MyTestClass(const std::string &_topic)
  {
    EXPECT_TRUE(this->node.Subscribe(_topic, &MyTestClass::Cb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void Cb(const IGN_T &_msg)
  {
    ros1_ign_bridge::testing::compareTestMsg(_msg);
    this->callbackExecuted = true;
  };

  /// \brief Reset the flags.
  public: void Reset()
  {
    this->callbackExecuted = false;
  }

  /// \brief Member variables that flag when the actions are executed.
  public: bool callbackExecuted = false;

  /// \brief Transport node;
  private: ignition::transport::Node node;
};

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Header)
{
  MyTestClass<ignition::msgs::Header> client("header");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, String)
{
  MyTestClass<ignition::msgs::StringMsg> client("string");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Quaternion)
{
  MyTestClass<ignition::msgs::Quaternion> client("quaternion");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Vector3)
{
  MyTestClass<ignition::msgs::Vector3d> client("vector3");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Image)
{
  MyTestClass<ignition::msgs::Image> client("image");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Imu)
{
  MyTestClass<ignition::msgs::IMU> client("imu");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, LaserScan)
{
  MyTestClass<ignition::msgs::LaserScan> client("laserscan");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Magnetometer)
{
  MyTestClass<ignition::msgs::Magnetometer> client("magnetic");

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ign_string_subscriber");

  return RUN_ALL_TESTS();
}
