/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <sensor_msgs/Image.h>
#include <chrono>
#include "../test_utils.h"

//////////////////////////////////////////////////
/// \brief A class for testing ROS topic subscription.
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
    ros_ign_image::testing::compareTestMsg(_msg);
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
TEST(ROSSubscriberTest, Image)
{
  MyTestClass<sensor_msgs::Image> client("image");

  using namespace std::chrono_literals;
  ros_ign_image::testing::waitUntilBoolVarAndSpin(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_image_subscriber");

  return RUN_ALL_TESTS();
}
