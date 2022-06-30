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

#ifndef SUBSCRIBERS__ROS_SUBSCRIBER_HPP_
#define SUBSCRIBERS__ROS_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "utils/test_utils.hpp"
#include "utils/ros_test_msg.hpp"

namespace ros_subscriber
{
/////////////////////////////////////////////////
/// \brief Retrieve a common node used for testing
rclcpp::Node * TestNode();

/////////////////////////////////////////////////
/// \brief A class for testing ROS topic subscription.
template<typename ROS_T>
class MyTestClass
{
public:
  /// \brief Class constructor.
  explicit MyTestClass(const std::string & _topic)
  {
    using std::placeholders::_1;
    this->sub = TestNode()->create_subscription<ROS_T>(
      _topic, 1000,
      std::bind(&MyTestClass::Cb, this, _1));
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


}  // namespace ros_subscriber

#endif  // SUBSCRIBERS__ROS_SUBSCRIBER_HPP_
