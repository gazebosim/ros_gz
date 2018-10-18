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

#ifndef ROS1_IGN_BRIDGE__TEST_UTILS_HH_
#define ROS1_IGN_BRIDGE__TEST_UTILS_HH_

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
#include <thread>
#include <ignition/common/Image.hh>
#include <ignition/msgs.hh>

namespace ros1_ign_bridge
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
  /// ROS 1 test utils
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

    EXPECT_GE(_msg.seq,         0);
    EXPECT_EQ(2,                _msg.stamp.sec);
    EXPECT_EQ(3,                _msg.stamp.nsec);
    EXPECT_EQ("frame_id_value", _msg.frame_id);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(std_msgs::String &_msg)
  {
    _msg.data = "string";
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::String &_msg)
  {
    std_msgs::String expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ("string", _msg.data);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::Quaternion &_msg)
  {
    _msg.x = 1;
    _msg.y = 2;
    _msg.z = 3;
    _msg.w = 4;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::Quaternion &_msg)
  {
    geometry_msgs::Quaternion expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(1, _msg.x);
    EXPECT_EQ(2, _msg.y);
    EXPECT_EQ(3, _msg.z);
    EXPECT_EQ(4, _msg.w);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::Vector3 &_msg)
  {
    _msg.x = 1;
    _msg.y = 2;
    _msg.z = 3;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::Vector3 &_msg)
  {
    geometry_msgs::Vector3 expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(1, _msg.x);
    EXPECT_EQ(2, _msg.y);
    EXPECT_EQ(3, _msg.z);
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

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::Imu &_msg)
  {
    std_msgs::Header header_msg;
    geometry_msgs::Quaternion quaternion_msg;
    geometry_msgs::Vector3 vector3_msg;

    createTestMsg(header_msg);
    createTestMsg(quaternion_msg);
    createTestMsg(vector3_msg);

    _msg.header                         = header_msg;
    _msg.orientation                    = quaternion_msg;
    _msg.orientation_covariance         = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    _msg.angular_velocity               = vector3_msg;
    _msg.angular_velocity_covariance    = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    _msg.linear_acceleration            = vector3_msg;
    _msg.linear_acceleration_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::Imu &_msg)
  {
    sensor_msgs::Imu expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    compareTestMsg(_msg.orientation);
    compareTestMsg(_msg.angular_velocity);
    compareTestMsg(_msg.linear_acceleration);

    for (auto i = 0; i < 9; ++i)
    {
      EXPECT_FLOAT_EQ(0, _msg.orientation_covariance[i]);
      EXPECT_FLOAT_EQ(0, _msg.angular_velocity_covariance[i]);
      EXPECT_FLOAT_EQ(0, _msg.linear_acceleration_covariance[i]);
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::LaserScan &_msg)
  {
    const unsigned int num_readings = 100u;
    const double laser_frequency    = 40;

    std_msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.header          = header_msg;
    _msg.angle_min       = -1.57;
    _msg.angle_max       = 1.57;
    _msg.angle_increment = 3.14 / num_readings;
    _msg.time_increment  = (1 / laser_frequency) / (num_readings);
    _msg.scan_time       = 0;
    _msg.range_min       = 1;
    _msg.range_max       = 2;
    _msg.ranges.resize(num_readings, 0);
    _msg.intensities.resize(num_readings, 1);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::LaserScan &_msg)
  {
    sensor_msgs::LaserScan expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    EXPECT_FLOAT_EQ(expected_msg.angle_min,       _msg.angle_min);
    EXPECT_FLOAT_EQ(expected_msg.angle_max,       _msg.angle_max);
    EXPECT_FLOAT_EQ(expected_msg.angle_increment, _msg.angle_increment);
    EXPECT_FLOAT_EQ(0,                            _msg.time_increment);
    EXPECT_FLOAT_EQ(0,                            _msg.scan_time);
    EXPECT_FLOAT_EQ(expected_msg.range_min,       _msg.range_min);
    EXPECT_FLOAT_EQ(expected_msg.range_max,       _msg.range_max);

    const unsigned int num_readings =
      (_msg.angle_max - _msg.angle_min) / _msg.angle_increment;
    for (auto i = 0u; i < num_readings; ++i)
    {
      EXPECT_FLOAT_EQ(expected_msg.ranges[i],      _msg.ranges[i]);
      EXPECT_FLOAT_EQ(expected_msg.intensities[i], _msg.intensities[i]);
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::MagneticField &_msg)
  {
    std_msgs::Header header_msg;
    geometry_msgs::Vector3 vector3_msg;

    createTestMsg(header_msg);
    createTestMsg(vector3_msg);

    _msg.header                    = header_msg;
    _msg.magnetic_field            = vector3_msg;
    _msg.magnetic_field_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::MagneticField &_msg)
  {
    sensor_msgs::MagneticField expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    compareTestMsg(_msg.magnetic_field);

    for (auto i = 0u; i < 9; ++i)
      EXPECT_FLOAT_EQ(0, _msg.magnetic_field_covariance[i]);
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
    EXPECT_EQ(2u,                            _msg.data_size());
    EXPECT_EQ(expected_msg.data(0).key(),    _msg.data(0).key());
    EXPECT_EQ(1u,                            _msg.data(0).value_size());
    std::string value = _msg.data(0).value(0);
    try
    {
      unsigned long ul = std::stoul(value, nullptr);
      EXPECT_GE(ul, 0);
    }
    catch (std::exception & e)
    {
      FAIL();
    }
    EXPECT_EQ(expected_msg.data(1).key(),    _msg.data(1).key());
    EXPECT_EQ(1u,                            _msg.data(1).value_size());
    EXPECT_EQ(expected_msg.data(1).value(0), _msg.data(1).value(0));

  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::StringMsg &_msg)
  {
    _msg.set_data("string");
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::StringMsg &_msg)
  {
    ignition::msgs::StringMsg expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.data(), _msg.data());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Quaternion &_msg)
  {
    _msg.set_x(1.0);
    _msg.set_y(2.0);
    _msg.set_z(3.0);
    _msg.set_w(4.0);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Quaternion &_msg)
  {
    ignition::msgs::Quaternion expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.x(), _msg.x());
    EXPECT_EQ(expected_msg.y(), _msg.y());
    EXPECT_EQ(expected_msg.z(), _msg.z());
    EXPECT_EQ(expected_msg.w(), _msg.w());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Vector3d &_msg)
  {
    _msg.set_x(1.0);
    _msg.set_y(2.0);
    _msg.set_z(3.0);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Vector3d &_msg)
  {
    ignition::msgs::Vector3d expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.x(), _msg.x());
    EXPECT_EQ(expected_msg.y(), _msg.y());
    EXPECT_EQ(expected_msg.z(), _msg.z());
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
    _msg.set_pixel_format(ignition::common::Image::PixelFormatType::RGB_INT8);
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
    EXPECT_EQ(expected_msg.width(),        _msg.width());
    EXPECT_EQ(expected_msg.height(),       _msg.height());
    EXPECT_EQ(expected_msg.pixel_format(), _msg.pixel_format());
    EXPECT_EQ(expected_msg.step(),         _msg.step());
    EXPECT_EQ(expected_msg.data(),         _msg.data());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::IMU &_msg)
  {
    ignition::msgs::Header header_msg;
    ignition::msgs::Quaternion quaternion_msg;
    ignition::msgs::Vector3d vector3_msg;

    createTestMsg(header_msg);
    createTestMsg(quaternion_msg);
    createTestMsg(vector3_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.mutable_orientation()->CopyFrom(quaternion_msg);
    _msg.mutable_angular_velocity()->CopyFrom(vector3_msg);
    _msg.mutable_linear_acceleration()->CopyFrom(vector3_msg);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::IMU &_msg)
  {
    ignition::msgs::IMU expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());
    compareTestMsg(_msg.orientation());
    compareTestMsg(_msg.angular_velocity());
    compareTestMsg(_msg.linear_acceleration());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::LaserScan &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);

    const unsigned int num_readings = 100u;
    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.set_frame("frame_id_value");
    _msg.set_angle_min(-1.57);
    _msg.set_angle_max(1.57);
    _msg.set_angle_step(3.14 / num_readings);
    _msg.set_range_min(1);
    _msg.set_range_max(2);
    _msg.set_count(num_readings);
    _msg.set_vertical_angle_min(0);
    _msg.set_vertical_angle_max(0);
    _msg.set_vertical_angle_step(0);
    _msg.set_vertical_count(0);

    for (auto i = 0u; i < _msg.count(); ++i)
    {
      _msg.add_ranges(0);
      _msg.add_intensities(1);
    }
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::LaserScan &_msg)
  {
    ignition::msgs::LaserScan expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());
    EXPECT_FLOAT_EQ(expected_msg.angle_min(),  _msg.angle_min());
    EXPECT_FLOAT_EQ(expected_msg.angle_max(),  _msg.angle_max());
    EXPECT_FLOAT_EQ(expected_msg.angle_step(), _msg.angle_step());
    EXPECT_DOUBLE_EQ(expected_msg.range_min(), _msg.range_min());
    EXPECT_DOUBLE_EQ(expected_msg.range_max(), _msg.range_max());
    EXPECT_EQ(expected_msg.count(),            _msg.count());
    EXPECT_DOUBLE_EQ(expected_msg.vertical_angle_min(),
                     _msg.vertical_angle_min());
    EXPECT_DOUBLE_EQ(expected_msg.vertical_angle_max(),
                     _msg.vertical_angle_max());
    EXPECT_DOUBLE_EQ(expected_msg.vertical_angle_step(),
                     _msg.vertical_angle_step());
    EXPECT_EQ(expected_msg.vertical_count(),   _msg.vertical_count());
    EXPECT_EQ(expected_msg.ranges_size(),      _msg.ranges_size());
    EXPECT_EQ(expected_msg.intensities_size(), _msg.intensities_size());

    for (auto i = 0u; i < _msg.count(); ++i)
    {
      EXPECT_DOUBLE_EQ(expected_msg.ranges(i),      _msg.ranges(i));
      EXPECT_DOUBLE_EQ(expected_msg.intensities(i), _msg.intensities(i));
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Magnetometer &_msg)
  {
    ignition::msgs::Header header_msg;
    ignition::msgs::Vector3d vector3_msg;

    createTestMsg(header_msg);
    createTestMsg(vector3_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.mutable_field_tesla()->CopyFrom(vector3_msg);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Magnetometer &_msg)
  {
    ignition::msgs::Magnetometer expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());
    compareTestMsg(_msg.field_tesla());
  }
}
}

#endif  // header guard
