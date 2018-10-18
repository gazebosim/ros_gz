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
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <chrono>
#include "../test_config.h"

//////////////////////////////////////////////////
/// \brief A class for testing ROS 1 topic subscription.
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
    this->sub = this->n.subscribe("header", 1000, &MyTestClass::HeaderCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void HeaderCb(const std_msgs::Header::ConstPtr& msg)
  {
    EXPECT_EQ(1u, msg->seq);
    EXPECT_EQ(2, msg->stamp.sec);
    EXPECT_EQ(3, msg->stamp.nsec);
    EXPECT_EQ("frame_id_value", msg->frame_id);
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeString()
  {
    this->sub = this->n.subscribe("string", 1000, &MyTestClass::StringCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void StringCb(const std_msgs::String::ConstPtr& msg)
  {
    EXPECT_EQ("string", msg->data);
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeQuaternion()
  {
    this->sub = this->n.subscribe(
      "quaternion", 1000, &MyTestClass::QuaternionCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void QuaternionCb(const geometry_msgs::Quaternion::ConstPtr& msg)
  {
    EXPECT_EQ(1, msg->x);
    EXPECT_EQ(2, msg->y);
    EXPECT_EQ(3, msg->z);
    EXPECT_EQ(4, msg->w);
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeVector3()
  {
    this->sub = this->n.subscribe(
      "vector3", 1000, &MyTestClass::Vector3Cb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void Vector3Cb(const geometry_msgs::Vector3::ConstPtr& msg)
  {
    EXPECT_EQ(1, msg->x);
    EXPECT_EQ(2, msg->y);
    EXPECT_EQ(3, msg->z);
    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeImage()
  {
    this->sub = this->n.subscribe("image", 1000, &MyTestClass::ImageCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void ImageCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    const unsigned int expected_width = 320u;
    const unsigned int expected_height = 240u;
    EXPECT_EQ(2, msg->header.stamp.sec);
    EXPECT_EQ(3, msg->header.stamp.nsec);
    EXPECT_EQ(expected_width, msg->width);
    EXPECT_EQ(expected_height, msg->height);
    EXPECT_EQ("rgb8", msg->encoding);
    EXPECT_FALSE(msg->is_bigendian);
    EXPECT_EQ(expected_width * 3, msg->step);

    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeImu()
  {
    this->sub = this->n.subscribe("imu", 1000, &MyTestClass::ImuCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void ImuCb(const sensor_msgs::Imu::ConstPtr& msg)
  {
    EXPECT_EQ(2, msg->header.stamp.sec);
    EXPECT_EQ(3, msg->header.stamp.nsec);
    EXPECT_EQ(1, msg->orientation.x);
    EXPECT_EQ(2, msg->orientation.y);
    EXPECT_EQ(3, msg->orientation.z);
    EXPECT_EQ(4, msg->orientation.w);
    EXPECT_EQ(1, msg->angular_velocity.x);
    EXPECT_EQ(2, msg->angular_velocity.y);
    EXPECT_EQ(3, msg->angular_velocity.z);
    EXPECT_EQ(1, msg->linear_acceleration.x);
    EXPECT_EQ(2, msg->linear_acceleration.y);
    EXPECT_EQ(3, msg->linear_acceleration.z);

    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeLaserScan()
  {
    this->sub = this->n.subscribe(
      "laserscan", 1000, &MyTestClass::LaserScanCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void LaserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    const unsigned int expected_num_readings = 100u;
    EXPECT_EQ(2, msg->header.stamp.sec);
    EXPECT_EQ(3, msg->header.stamp.nsec);
    EXPECT_FLOAT_EQ(-1.57, msg->angle_min);
    EXPECT_FLOAT_EQ(1.57, msg->angle_max);
    EXPECT_FLOAT_EQ(3.14 / expected_num_readings, msg->angle_increment);
    EXPECT_FLOAT_EQ(0, msg->time_increment);
    EXPECT_FLOAT_EQ(0, msg->scan_time);
    EXPECT_FLOAT_EQ(1, msg->range_min);
    EXPECT_FLOAT_EQ(2, msg->range_max);

    this->callbackExecuted = true;
  };

  /// \brief Create a subscriber.
  public: void SubscribeMagneticField()
  {
    this->sub = this->n.subscribe(
      "magnetic", 1000, &MyTestClass::MagneticFieldCb, this);
  }

  /// \brief Member function called each time a topic update is received.
  public: void MagneticFieldCb(const sensor_msgs::MagneticField::ConstPtr& msg)
  {
    EXPECT_EQ(2, msg->header.stamp.sec);
    EXPECT_EQ(3, msg->header.stamp.nsec);
    EXPECT_EQ(1, msg->magnetic_field.x);
    EXPECT_EQ(2, msg->magnetic_field.y);
    EXPECT_EQ(3, msg->magnetic_field.z);
    for (auto i = 0; i < 9; i++)
      EXPECT_EQ(0, msg->magnetic_field_covariance[i]);

    this->callbackExecuted = true;
  };

  public: void Reset()
  {
    this->callbackExecuted = false;
  }

  /// \brief Member variables that flag when the actions are executed.
  public: bool callbackExecuted;

  /// \brief ROS node handle;
  private: ros::NodeHandle n;

  /// \brief ROS subscriber;
  private: ros::Subscriber sub;
};

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Header)
{
  MyTestClass client;
  client.SubscribeHeader();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, String)
{
  MyTestClass client;
  client.SubscribeString();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Quaternion)
{
  MyTestClass client;
  client.SubscribeQuaternion();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Vector3)
{
  MyTestClass client;
  client.SubscribeVector3();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Image)
{
  MyTestClass client;
  client.SubscribeImage();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, Imu)
{
  MyTestClass client;
  client.SubscribeImu();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, LaserScan)
{
  MyTestClass client;
  client.SubscribeLaserScan();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROS1SubscriberTest, MagneticField)
{
  MyTestClass client;
  client.SubscribeMagneticField();

  using namespace std::chrono_literals;
  ros1_ign_bridge::testing::waitUntilBoolVarAndSpin(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros1_string_subscriber");

  return RUN_ALL_TESTS();
}
