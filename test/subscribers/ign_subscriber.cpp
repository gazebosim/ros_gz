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
#include <chrono>
#include <ignition/common/Image.hh>
#include <ignition/transport.hh>
#include "../test_config.h"

//////////////////////////////////////////////////
/// \brief A class for testing Ignition Transport topic subscription.
class MyTestClass
{
  /// \brief Class constructor.
  public: MyTestClass()
    : callbackExecuted(false)
  {
  }

  /// \brief Create a subscriber.
  public: void SubscribeHeader()
  {
    EXPECT_TRUE(this->node.Subscribe("header", &MyTestClass::HeaderCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void HeaderCb(const ignition::msgs::Header &_msg)
  {
    EXPECT_EQ(2, _msg.stamp().sec());
    EXPECT_EQ(3, _msg.stamp().nsec());

    ASSERT_EQ(2, _msg.data_size());
    EXPECT_EQ("seq", _msg.data(0).key());
    ASSERT_EQ(1, _msg.data(0).value_size());
    EXPECT_EQ("1", _msg.data(0).value(0));
    EXPECT_EQ("frame_id", _msg.data(1).key());
    ASSERT_EQ(1, _msg.data(1).value_size());
    EXPECT_EQ("frame_id_value", _msg.data(1).value(0));
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeString()
  {
    EXPECT_TRUE(this->node.Subscribe("string", &MyTestClass::StringCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void StringCb(const ignition::msgs::StringMsg &_msg)
  {
    EXPECT_EQ("string", _msg.data());
    this->callbackExecuted = true;
  };

  public: void Reset()
  {
    this->callbackExecuted = false;
  }

  /// \brief Create a subscriber.
  public: void SubscribeQuaternion()
  {
    EXPECT_TRUE(this->node.Subscribe("quaternion",
      &MyTestClass::QuaternionCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void QuaternionCb(const ignition::msgs::Quaternion &_msg)
  {
    EXPECT_EQ(1, _msg.x());
    EXPECT_EQ(2, _msg.y());
    EXPECT_EQ(3, _msg.z());
    EXPECT_EQ(4, _msg.w());
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeVector3()
  {
    EXPECT_TRUE(this->node.Subscribe("vector3", &MyTestClass::Vector3Cb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void Vector3Cb(const ignition::msgs::Vector3d &_msg)
  {
    EXPECT_EQ(1, _msg.x());
    EXPECT_EQ(2, _msg.y());
    EXPECT_EQ(3, _msg.z());
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeImage()
  {
    EXPECT_TRUE(this->node.Subscribe("image", &MyTestClass::ImageCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void ImageCb(const ignition::msgs::Image &_msg)
  {
    const unsigned int expectedWidth = 320u;
    const unsigned int expectedHeight = 240u;
    EXPECT_EQ(expectedWidth, _msg.width());
    EXPECT_EQ(expectedHeight, _msg.height());
    EXPECT_EQ(ignition::common::Image::PixelFormatType::RGB_INT8,
      _msg.pixel_format());
    EXPECT_EQ(expectedWidth * 3, _msg.step());
    EXPECT_EQ(expectedHeight * _msg.step(), _msg.data().size());
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeImu()
  {
    EXPECT_TRUE(this->node.Subscribe("imu", &MyTestClass::ImuCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void ImuCb(const ignition::msgs::IMU &_msg)
  {
    EXPECT_EQ(1, _msg.orientation().x());
    EXPECT_EQ(2, _msg.orientation().y());
    EXPECT_EQ(3, _msg.orientation().z());
    EXPECT_EQ(4, _msg.orientation().w());
    EXPECT_EQ(1, _msg.angular_velocity().x());
    EXPECT_EQ(2, _msg.angular_velocity().y());
    EXPECT_EQ(3, _msg.angular_velocity().z());
    EXPECT_EQ(1, _msg.linear_acceleration().x());
    EXPECT_EQ(2, _msg.linear_acceleration().y());
    EXPECT_EQ(3, _msg.linear_acceleration().z());
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeLaserScan()
  {
    EXPECT_TRUE(this->node.Subscribe("laserscan", &MyTestClass::LaserCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void LaserCb(const ignition::msgs::LaserScan &_msg)
  {
    const unsigned int expected_num_readings = 100u;
    EXPECT_FLOAT_EQ(-1.57, _msg.angle_min());
    EXPECT_FLOAT_EQ(1.57, _msg.angle_max());
    EXPECT_FLOAT_EQ(3.14 / expected_num_readings, _msg.angle_step());
    EXPECT_DOUBLE_EQ(1, _msg.range_min());
    EXPECT_DOUBLE_EQ(2, _msg.range_max());
    EXPECT_EQ(expected_num_readings, _msg.count());
    EXPECT_DOUBLE_EQ(0, _msg.vertical_angle_min());
    EXPECT_DOUBLE_EQ(0, _msg.vertical_angle_max());
    EXPECT_DOUBLE_EQ(0, _msg.vertical_angle_step());
    EXPECT_EQ(0, _msg.vertical_count());
    EXPECT_EQ(expected_num_readings, _msg.ranges_size());
    EXPECT_EQ(expected_num_readings, _msg.intensities_size());
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeMagnetometer()
  {
    EXPECT_TRUE(this->node.Subscribe(
      "magnetic", &MyTestClass::MagnetometerCb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void MagnetometerCb(const ignition::msgs::Magnetometer &_msg)
  {
    EXPECT_EQ(2, _msg.header().stamp().sec());
    EXPECT_EQ(3, _msg.header().stamp().nsec());
    EXPECT_DOUBLE_EQ(1, _msg.field_tesla().x());
    EXPECT_DOUBLE_EQ(2, _msg.field_tesla().y());
    EXPECT_DOUBLE_EQ(3, _msg.field_tesla().z());
    this->callbackExecuted = true;
  };

  /// \brief Member variables that flag when the actions are executed.
  public: bool callbackExecuted;

  /// \brief Transport node;
  private: ignition::transport::Node node;
};

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Header)
{
  MyTestClass client;
  client.SubscribeHeader();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, String)
{
  MyTestClass client;
  client.SubscribeString();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Quaternion)
{
  MyTestClass client;
  client.SubscribeQuaternion();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Vector3)
{
  MyTestClass client;
  client.SubscribeVector3();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Image)
{
  MyTestClass client;
  client.SubscribeImage();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Imu)
{
  MyTestClass client;
  client.SubscribeImu();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, LaserScan)
{
  MyTestClass client;
  client.SubscribeLaserScan();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, Magnetometer)
{
  MyTestClass client;
  client.SubscribeMagnetometer();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ign_string_subscriber");

  return RUN_ALL_TESTS();
}
