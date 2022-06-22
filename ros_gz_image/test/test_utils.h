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

#ifndef ROS_IGN_IMAGE__TEST_UTILS_H_
#define ROS_IGN_IMAGE__TEST_UTILS_H_

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include <string>
#include <thread>
#include <ignition/msgs.hh>

namespace ros_ign_image
{
namespace testing
{
  /// \brief Wait until a boolean variable is set to true for a given number
  /// of times.
  /// \param[in out] _boolVar The bool variable.
  /// \param[in] _sleepEach Time duration to wait between each retry.
  /// \param[in] _retries The number of retries.
  ///
  /// E.g.:
  ///   using namespace std::chrono_literals;
  ///   waitUntilBoolVar(myVar, 1ms, 10);
  template <class Rep, class Period>
  void waitUntilBoolVar(
      bool &_boolVar,
      const std::chrono::duration<Rep, Period> &_sleepEach,
      const int _retries)
  {
    int i = 0;
    while (!_boolVar && i < _retries)
    {
      ++i;
      std::this_thread::sleep_for(_sleepEach);
    }
  }

  /// \brief Wait until a boolean variable is set to true for a given number
  /// of times. This function calls ros::spinOnce each iteration.
  /// \param[in out] _boolVar The bool variable.
  /// \param[in] _sleepEach Time duration to wait between each retry.
  /// \param[in] _retries The number of retries.
  ///
  /// E.g.:
  ///   using namespace std::chrono_literals;
  ///   waitUntilBoolVar(myVar, 1ms, 10);
  template <class Rep, class Period>
  void waitUntilBoolVarAndSpin(
      bool &_boolVar,
      const std::chrono::duration<Rep, Period> &_sleepEach,
      const int _retries)
  {
    int i = 0;
    while (!_boolVar && i < _retries)
    {
      ++i;
      std::this_thread::sleep_for(_sleepEach);
      ros::spinOnce();
    }
  }

  //////////////////////////////////////////////////
  /// ROS test utils
  //////////////////////////////////////////////////

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(std_msgs::Header &_msg)
  {
    _msg.seq        = 1;
    _msg.stamp.sec  = 2;
    _msg.stamp.nsec = 3;
    _msg.frame_id   = "frame_id_value";
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::Header &_msg)
  {
    std_msgs::Header expected_msg;
    createTestMsg(expected_msg);

    EXPECT_GE(expected_msg.seq,        0u);
    EXPECT_EQ(expected_msg.stamp.sec,  _msg.stamp.sec);
    EXPECT_EQ(expected_msg.stamp.nsec, _msg.stamp.nsec);
    EXPECT_EQ(expected_msg.frame_id,   _msg.frame_id);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::Image &_msg)
  {
    std_msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.header       = header_msg;
    _msg.width        = 320;
    _msg.height       = 240;
    _msg.encoding     = "rgb8";
    _msg.is_bigendian = false;
    _msg.step         = _msg.width * 3;
    _msg.data.resize(_msg.height * _msg.step, '1');
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::Image &_msg)
  {
    sensor_msgs::Image expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    EXPECT_EQ(expected_msg.width,        _msg.width);
    EXPECT_EQ(expected_msg.height,       _msg.height);
    EXPECT_EQ(expected_msg.encoding,     _msg.encoding);
    EXPECT_EQ(expected_msg.is_bigendian, _msg.is_bigendian);
    EXPECT_EQ(expected_msg.step,         _msg.step);
  }

  //////////////////////////////////////////////////
  /// Ignition::msgs test utils
  //////////////////////////////////////////////////

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Header &_msg)
  {
    auto seq_entry = _msg.add_data();
    seq_entry->set_key("seq");
    seq_entry->add_value("1");
    _msg.mutable_stamp()->set_sec(2);
    _msg.mutable_stamp()->set_nsec(3);
    auto frame_id_entry = _msg.add_data();
    frame_id_entry->set_key("frame_id");
    frame_id_entry->add_value("frame_id_value");
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Header &_msg)
  {
    ignition::msgs::Header expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.stamp().sec(),    _msg.stamp().sec());
    EXPECT_EQ(expected_msg.stamp().nsec(),   _msg.stamp().nsec());
    EXPECT_GE(_msg.data_size(),              2);
    EXPECT_EQ(expected_msg.data(0).key(),    _msg.data(0).key());
    EXPECT_EQ(1,                             _msg.data(0).value_size());
    std::string value = _msg.data(0).value(0);
    try
    {
      uint32_t ul = std::stoul(value, nullptr);
      EXPECT_GE(ul, 0u);
    }
    catch (std::exception & e)
    {
      FAIL();
    }
    EXPECT_EQ(expected_msg.data(1).key(),    _msg.data(1).key());
    EXPECT_EQ(1,                             _msg.data(1).value_size());
    EXPECT_EQ(expected_msg.data(1).value(0), _msg.data(1).value(0));
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Image &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.set_width(320);
    _msg.set_height(240);
    _msg.set_pixel_format_type(ignition::msgs::PixelFormatType::RGB_INT8);
    _msg.set_step(_msg.width() * 3);
    _msg.set_data(std::string(_msg.height() * _msg.step(), '1'));
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Image &_msg)
  {
    ignition::msgs::Image expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());
    EXPECT_EQ(expected_msg.width(),             _msg.width());
    EXPECT_EQ(expected_msg.height(),            _msg.height());
    EXPECT_EQ(expected_msg.pixel_format_type(), _msg.pixel_format_type());
    EXPECT_EQ(expected_msg.step(),              _msg.step());
    EXPECT_EQ(expected_msg.data(),              _msg.data());
  }
}
}

#endif  // ROS_IGN_BRIDGE__TEST_UTILS_H_
