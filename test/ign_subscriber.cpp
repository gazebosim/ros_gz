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
#include <ignition/transport.hh>
#include "test_config.h"

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
  public: void Subscribe()
  {
    EXPECT_TRUE(this->node.Subscribe("/chatter", &MyTestClass::Cb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void Cb(const ignition::msgs::StringMsg &/*_msg*/)
  {
    this->callbackExecuted = true;
  };

  public: void Reset()
  {
    this->callbackExecuted = false;
  }

  /// \brief Member variables that flag when the actions are executed.
  public: bool callbackExecuted;

  /// \brief Transport node;
  private: ignition::transport::Node node;
};

/////////////////////////////////////////////////
TEST(IgnSubscriberTest, String)
{
  MyTestClass client;
  client.Subscribe();

  using namespace std::chrono_literals;
  ros1_ign_bridge::waitUntilBoolVar(client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ign_subscriber");

  return RUN_ALL_TESTS();
}
