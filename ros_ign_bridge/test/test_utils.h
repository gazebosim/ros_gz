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

#ifndef ROS_IGN_BRIDGE__TEST_UTILS_H_
#define ROS_IGN_BRIDGE__TEST_UTILS_H_

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <string>
#include <thread>
#include <ignition/msgs.hh>

namespace ros_ign_bridge
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
  void createTestMsg(std_msgs::Bool &_msg)
  {
    _msg.data = true;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::Bool &_msg)
  {
    std_msgs::Bool expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.data, _msg.data);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(std_msgs::ColorRGBA &_msg)
  {
    _msg.r = 10.0;
    _msg.g = 11.0;
    _msg.b = 12.0;
    _msg.a = 13.0;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::ColorRGBA &_msg)
  {
    std_msgs::ColorRGBA expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.r, _msg.r);
    EXPECT_EQ(expected_msg.g, _msg.g);
    EXPECT_EQ(expected_msg.b, _msg.b);
    EXPECT_EQ(expected_msg.a, _msg.a);
  }

  /// \brief Compare a message with the populated for testing. Noop for Empty
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::Empty &)
  {
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(std_msgs::Int32 &_msg)
  {
    _msg.data = 5;
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(std_msgs::Float32 &_msg)
  {
    _msg.data = 1.5;
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(std_msgs::Float64 &_msg)
  {
    _msg.data = 1.5;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::Int32 &_msg)
  {
    std_msgs::Int32 expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.data, _msg.data);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::Float32 &_msg)
  {
    std_msgs::Float32 expected_msg;
    createTestMsg(expected_msg);

    EXPECT_FLOAT_EQ(expected_msg.data, _msg.data);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const std_msgs::Float64 &_msg)
  {
    std_msgs::Float64 expected_msg;
    createTestMsg(expected_msg);

    EXPECT_DOUBLE_EQ(expected_msg.data, _msg.data);
  }

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

    EXPECT_EQ(expected_msg.data, _msg.data);
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

    EXPECT_EQ(expected_msg.x, _msg.x);
    EXPECT_EQ(expected_msg.y, _msg.y);
    EXPECT_EQ(expected_msg.z, _msg.z);
    EXPECT_EQ(expected_msg.w, _msg.w);
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

    EXPECT_EQ(expected_msg.x, _msg.x);
    EXPECT_EQ(expected_msg.y, _msg.y);
    EXPECT_EQ(expected_msg.z, _msg.z);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(rosgraph_msgs::Clock &_msg)
  {
    _msg.clock.sec  = 1;
    _msg.clock.nsec = 2;
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::Point &_msg)
  {
    _msg.x = 1;
    _msg.y = 2;
    _msg.z = 3;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const rosgraph_msgs::Clock &_msg)
  {
    rosgraph_msgs::Clock expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.clock.sec, _msg.clock.sec);
    EXPECT_EQ(expected_msg.clock.nsec, _msg.clock.nsec);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::Point &_msg)
  {
    geometry_msgs::Point expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.x, _msg.x);
    EXPECT_EQ(expected_msg.y, _msg.y);
    EXPECT_EQ(expected_msg.z, _msg.z);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::Pose &_msg)
  {
    createTestMsg(_msg.position);
    createTestMsg(_msg.orientation);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::Pose &_msg)
  {
    compareTestMsg(_msg.position);
    compareTestMsg(_msg.orientation);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::PoseArray &_msg)
  {
    geometry_msgs::Pose pose;
    createTestMsg(pose);
    _msg.poses.push_back(pose);

    createTestMsg(_msg.header);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::PoseArray &_msg)
  {
    compareTestMsg(_msg.poses[0]);
    compareTestMsg(_msg.header);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::PoseStamped &_msg)
  {
    createTestMsg(_msg.header);
    createTestMsg(_msg.pose);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::PoseStamped &_msg)
  {
    compareTestMsg(_msg.header);
    compareTestMsg(_msg.pose);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::Transform &_msg)
  {
    createTestMsg(_msg.translation);
    createTestMsg(_msg.rotation);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::Transform &_msg)
  {
    compareTestMsg(_msg.translation);
    compareTestMsg(_msg.rotation);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::TransformStamped &_msg)
  {
    createTestMsg(_msg.header);
    createTestMsg(_msg.transform);
    _msg.child_frame_id = "child_frame_id_value";
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::TransformStamped &_msg)
  {
    geometry_msgs::TransformStamped expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    compareTestMsg(_msg.transform);
    EXPECT_EQ(expected_msg.child_frame_id, _msg.child_frame_id);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(tf2_msgs::TFMessage &_msg)
  {
    geometry_msgs::TransformStamped tf;
    createTestMsg(tf);
    _msg.transforms.push_back(tf);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const tf2_msgs::TFMessage &_msg)
  {
    tf2_msgs::TFMessage expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.transforms[0]);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(geometry_msgs::Twist &_msg)
  {
    createTestMsg(_msg.linear);
    createTestMsg(_msg.angular);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const geometry_msgs::Twist &_msg)
  {
    compareTestMsg(_msg.linear);
    compareTestMsg(_msg.angular);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(mav_msgs::Actuators &_msg)
  {
    createTestMsg(_msg.header);
    for (auto i = 0u; i < 5; ++i)
    {
      _msg.angles.push_back(i);
      _msg.angular_velocities.push_back(i);
      _msg.normalized.push_back(i);
    }
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const mav_msgs::Actuators &_msg)
  {
    mav_msgs::Actuators expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);

    ASSERT_EQ(expected_msg.angles.size(), _msg.angles.size());
    ASSERT_EQ(expected_msg.angular_velocities.size(),
              _msg.angular_velocities.size());
    ASSERT_EQ(expected_msg.normalized.size(), _msg.normalized.size());

    for (auto i = 0u; i < _msg.angles.size(); ++i)
    {
      EXPECT_EQ(expected_msg.angles[i], _msg.angles[i]);
      EXPECT_EQ(expected_msg.angular_velocities[i], _msg.angular_velocities[i]);
      EXPECT_EQ(expected_msg.normalized[i], _msg.normalized[i]);
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(nav_msgs::OccupancyGrid &_msg)
  {
    createTestMsg(_msg.header);

    _msg.info.map_load_time.sec = 100;
    _msg.info.map_load_time.nsec = 200;
    _msg.info.resolution = 0.05;
    _msg.info.width = 10;
    _msg.info.height = 20;
    createTestMsg(_msg.info.origin);
    _msg.data.resize(_msg.info.width * _msg.info.height, 1);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const nav_msgs::OccupancyGrid &_msg)
  {
    nav_msgs::OccupancyGrid expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    EXPECT_EQ(expected_msg.info.map_load_time.sec,
              _msg.info.map_load_time.sec);
    EXPECT_EQ(expected_msg.info.map_load_time.nsec,
              _msg.info.map_load_time.nsec);
    EXPECT_FLOAT_EQ(expected_msg.info.resolution, _msg.info.resolution);
    EXPECT_EQ(expected_msg.info.width, _msg.info.width);
    EXPECT_EQ(expected_msg.info.height, _msg.info.height);

    compareTestMsg(_msg.info.origin);

    EXPECT_EQ(expected_msg.data.size(), _msg.data.size());
    EXPECT_EQ(expected_msg.data[0], _msg.data[0]);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(nav_msgs::Odometry &_msg)
  {
    createTestMsg(_msg.header);
    createTestMsg(_msg.pose.pose);
    createTestMsg(_msg.twist.twist);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const nav_msgs::Odometry &_msg)
  {
    compareTestMsg(_msg.header);
    compareTestMsg(_msg.pose.pose);
    compareTestMsg(_msg.twist.twist);
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
  void createTestMsg(sensor_msgs::CameraInfo &_msg)
  {
    std_msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.header       = header_msg;
    _msg.width        = 320;
    _msg.height       = 240;
    _msg.distortion_model = "plumb_bob";
    _msg.D.resize(5);
    _msg.D[0] = 1;
    _msg.D[1] = 2;
    _msg.D[2] = 3;
    _msg.D[3] = 4;
    _msg.D[4] = 5;

    _msg.K[0] = 1;
    _msg.K[1] = 0;
    _msg.K[2] = 0;
    _msg.K[3] = 0;
    _msg.K[4] = 1;
    _msg.K[5] = 0;
    _msg.K[6] = 0;
    _msg.K[7] = 0;
    _msg.K[8] = 1;

    _msg.R[0] = 1;
    _msg.R[1] = 0;
    _msg.R[2] = 0;
    _msg.R[3] = 0;
    _msg.R[4] = 1;
    _msg.R[5] = 0;
    _msg.R[6] = 0;
    _msg.R[7] = 0;
    _msg.R[8] = 1;

    _msg.P[0] = 1;
    _msg.P[1] = 0;
    _msg.P[2] = 0;
    _msg.P[3] = 0;
    _msg.P[4] = 0;
    _msg.P[5] = 1;
    _msg.P[6] = 0;
    _msg.P[7] = 0;
    _msg.P[8] = 0;
    _msg.P[9] = 0;
    _msg.P[10] = 1;
    _msg.P[11] = 0;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::CameraInfo &_msg)
  {
    sensor_msgs::CameraInfo expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    EXPECT_EQ(expected_msg.width, _msg.width);
    EXPECT_EQ(expected_msg.height, _msg.height);
    EXPECT_EQ(expected_msg.distortion_model, _msg.distortion_model);

    for (auto i = 0; i < 12; ++i)
    {
      EXPECT_EQ(expected_msg.P[i], _msg.P[i]);

      if (i > 8)
        continue;

      EXPECT_EQ(expected_msg.K[i], _msg.K[i]);
      EXPECT_EQ(expected_msg.R[i], _msg.R[i]);

      if (i > 4)
        continue;

      EXPECT_EQ(expected_msg.D[i], _msg.D[i]);
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::FluidPressure &_msg)
  {
    std_msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.header = header_msg;
    _msg.fluid_pressure = 0.123;
    _msg.variance = 0.456;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::FluidPressure &_msg)
  {
    sensor_msgs::FluidPressure expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    EXPECT_FLOAT_EQ(expected_msg.fluid_pressure, _msg.fluid_pressure);
    EXPECT_FLOAT_EQ(expected_msg.variance, _msg.variance);
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
  void createTestMsg(sensor_msgs::JointState &_msg)
  {
    std_msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.header   = header_msg;
    _msg.name     = {"joint_0", "joint_1", "joint_2"};
    _msg.position = {1, 1, 1};
    _msg.velocity = {2, 2, 2};
    _msg.effort   = {3, 3, 3};
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::JointState &_msg)
  {
    sensor_msgs::JointState expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);

    ASSERT_EQ(expected_msg.name.size(),     _msg.name.size());
    ASSERT_EQ(expected_msg.position.size(), _msg.position.size());
    ASSERT_EQ(expected_msg.velocity.size(), _msg.velocity.size());
    ASSERT_EQ(expected_msg.effort.size(),   _msg.effort.size());

    for (auto i = 0u; i < _msg.position.size(); ++i)
    {
      EXPECT_EQ(expected_msg.name[i],           _msg.name[i]);
      EXPECT_FLOAT_EQ(expected_msg.position[i], _msg.position[i]);
      EXPECT_FLOAT_EQ(expected_msg.velocity[i], _msg.velocity[i]);
      EXPECT_FLOAT_EQ(expected_msg.effort[i],   _msg.effort[i]);
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
    compareTestMsg(_msg.header);
    compareTestMsg(_msg.magnetic_field);

    for (auto i = 0u; i < 9; ++i)
      EXPECT_FLOAT_EQ(0, _msg.magnetic_field_covariance[i]);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::PointCloud2 &_msg)
  {
    createTestMsg(_msg.header);

    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    _msg.fields.push_back(field);

    uint32_t height = 4;
    uint32_t width = 2;

    _msg.height = height;
    _msg.width = width;
    _msg.is_bigendian = false;
    _msg.point_step = 4;
    _msg.row_step = 4 * width;
    _msg.is_dense = true;

    _msg.data.resize(_msg.row_step * _msg.height);
    uint8_t *msgBufferIndex = _msg.data.data();

    for (uint32_t j = 0; j < height; ++j)
    {
      for (uint32_t i = 0; i < width; ++i)
      {
        *reinterpret_cast<float*>(msgBufferIndex + _msg.fields[0].offset) =
          j * width + i;
        msgBufferIndex += _msg.point_step;
      }
    }
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::PointCloud2 &_msg)
  {
    compareTestMsg(_msg.header);

    uint32_t height = 4;
    uint32_t width = 2;

    EXPECT_EQ(height, _msg.height);
    EXPECT_EQ(width, _msg.width);
    EXPECT_FALSE(_msg.is_bigendian);
    EXPECT_EQ(4u, _msg.point_step);
    EXPECT_EQ(4U * width, _msg.row_step);
    EXPECT_TRUE(_msg.is_dense);

    unsigned char *msgBufferIndex =
      const_cast<unsigned char*>(_msg.data.data());

    for (uint32_t j = 0; j < height; ++j)
    {
      for (uint32_t i = 0; i < width; ++i)
      {
        float *value =
          reinterpret_cast<float*>(msgBufferIndex + _msg.fields[0].offset);

        EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
        msgBufferIndex += _msg.point_step;
      }
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(sensor_msgs::BatteryState &_msg)
  {
    std_msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.header = header_msg;
    _msg.voltage = 123;
    _msg.current = 456;
    _msg.charge = 789;
    _msg.capacity = 321;
    _msg.percentage = 654;
    _msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const sensor_msgs::BatteryState &_msg)
  {
    sensor_msgs::BatteryState expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);
    EXPECT_EQ(expected_msg.voltage, _msg.voltage);
    EXPECT_EQ(expected_msg.current, _msg.current);
    EXPECT_EQ(expected_msg.charge, _msg.charge);
    EXPECT_EQ(expected_msg.capacity, _msg.capacity);
    EXPECT_EQ(expected_msg.percentage, _msg.percentage);
    EXPECT_EQ(expected_msg.power_supply_status, _msg.power_supply_status);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(visualization_msgs::Marker &_msg)
  {
    createTestMsg(_msg.header);

    _msg.ns = "foo";
    _msg.id = 123;
    _msg.type = visualization_msgs::Marker::CUBE;
    _msg.action = visualization_msgs::Marker::ADD;

    createTestMsg(_msg.pose);
    createTestMsg(_msg.scale);
    createTestMsg(_msg.color);

    _msg.lifetime.sec = 100;
    _msg.lifetime.nsec = 200;

    geometry_msgs::Point pt;
    createTestMsg(pt);
    _msg.points.push_back(pt);
    _msg.text = "bar";
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const visualization_msgs::Marker &_msg)
  {
    visualization_msgs::Marker expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header);

    EXPECT_EQ(expected_msg.ns, _msg.ns);
    EXPECT_EQ(expected_msg.id, _msg.id);
    EXPECT_EQ(expected_msg.type, _msg.type);
    EXPECT_EQ(expected_msg.action, _msg.action);

    compareTestMsg(_msg.pose);
    compareTestMsg(_msg.scale);
    compareTestMsg(_msg.color);

    EXPECT_EQ(expected_msg.lifetime.sec, _msg.lifetime.sec);
    EXPECT_EQ(expected_msg.lifetime.nsec, _msg.lifetime.nsec);

    compareTestMsg(_msg.points[0]);

    EXPECT_EQ(expected_msg.text, _msg.text);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(visualization_msgs::MarkerArray &_msg)
  {
    _msg.markers.clear();
    visualization_msgs::Marker marker;
    createTestMsg(marker);
    _msg.markers.push_back(marker);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const visualization_msgs::MarkerArray &_msg)
  {
    visualization_msgs::MarkerArray expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(1u, _msg.markers.size());
    compareTestMsg(_msg.markers[0]);
  }

  //////////////////////////////////////////////////
  /// Ignition::msgs test utils
  //////////////////////////////////////////////////

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Boolean &_msg)
  {
    _msg.set_data(true);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Boolean &_msg)
  {
    ignition::msgs::Boolean expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.data(), _msg.data());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Color &_msg)
  {
    _msg.set_r(10.0);
    _msg.set_g(11.0);
    _msg.set_b(12.0);
    _msg.set_a(13.0);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Color &_msg)
  {
    ignition::msgs::Color expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.r(), _msg.r());
    EXPECT_EQ(expected_msg.g(), _msg.g());
    EXPECT_EQ(expected_msg.b(), _msg.b());
    EXPECT_EQ(expected_msg.a(), _msg.a());
  }

  /// \brief Compare a message with the populated for testing. Noop for Empty
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Empty &)
  {
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Int32 &_msg)
  {
    _msg.set_data(5);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Int32 &_msg)
  {
    ignition::msgs::Int32 expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.data(), _msg.data());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Float &_msg)
  {
    _msg.set_data(1.5);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Float &_msg)
  {
    ignition::msgs::Float expected_msg;
    createTestMsg(expected_msg);

    EXPECT_FLOAT_EQ(expected_msg.data(), _msg.data());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Double &_msg)
  {
    _msg.set_data(1.5);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Double &_msg)
  {
    ignition::msgs::Double expected_msg;
    createTestMsg(expected_msg);

    EXPECT_DOUBLE_EQ(expected_msg.data(), _msg.data());
  }

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
  void createTestMsg(ignition::msgs::Clock &_msg)
  {
    _msg.mutable_sim()->set_sec(1);
    _msg.mutable_sim()->set_nsec(2);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Clock &_msg)
  {
    ignition::msgs::Clock expected_msg;
    createTestMsg(expected_msg);

    EXPECT_EQ(expected_msg.sim().sec(),    _msg.sim().sec());
    EXPECT_EQ(expected_msg.sim().nsec(),   _msg.sim().nsec());
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
  void createTestMsg(ignition::msgs::Pose &_msg)
  {
    createTestMsg(*_msg.mutable_header());
    auto child_frame_id_entry = _msg.mutable_header()->add_data();
    child_frame_id_entry->set_key("child_frame_id");
    child_frame_id_entry->add_value("child_frame_id_value");

    createTestMsg(*_msg.mutable_position());
    createTestMsg(*_msg.mutable_orientation());
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Pose &_msg)
  {
    if (_msg.header().data_size() > 0)
    {
      compareTestMsg(_msg.header());

      ignition::msgs::Pose expected_msg;
      createTestMsg(expected_msg);

      if (_msg.header().data_size() > 2)
      {
        // child_frame_id
        ASSERT_EQ(3, expected_msg.header().data_size());
        ASSERT_EQ(3, _msg.header().data_size());
        EXPECT_EQ(expected_msg.header().data(2).key(),
          _msg.header().data(2).key());
        EXPECT_EQ(1, _msg.header().data(2).value_size());
        EXPECT_EQ(expected_msg.header().data(2).value(0),
            _msg.header().data(2).value(0));
      }
    }

    compareTestMsg(_msg.position());
    compareTestMsg(_msg.orientation());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Pose_V &_msg)
  {
    createTestMsg(*_msg.mutable_header());
    createTestMsg(*_msg.add_pose());
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Pose_V &_msg)
  {
    ignition::msgs::Pose_V expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());
    compareTestMsg(_msg.pose(0));
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Twist &_msg)
  {
    ignition::msgs::Header header_msg;
    ignition::msgs::Vector3d linear_msg;
    ignition::msgs::Vector3d angular_msg;

    createTestMsg(header_msg);
    createTestMsg(linear_msg);
    createTestMsg(angular_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.mutable_linear()->CopyFrom(linear_msg);
    _msg.mutable_angular()->CopyFrom(angular_msg);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Twist &_msg)
  {
    compareTestMsg(_msg.linear());
    compareTestMsg(_msg.angular());
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

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::CameraInfo &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.set_width(320);
    _msg.set_height(240);

    auto distortion = _msg.mutable_distortion();
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::PLUMB_BOB);
    distortion->add_k(1);
    distortion->add_k(2);
    distortion->add_k(3);
    distortion->add_k(4);
    distortion->add_k(5);

    auto intrinsics = _msg.mutable_intrinsics();
    intrinsics->add_k(1);
    intrinsics->add_k(0);
    intrinsics->add_k(0);
    intrinsics->add_k(0);
    intrinsics->add_k(1);
    intrinsics->add_k(0);
    intrinsics->add_k(0);
    intrinsics->add_k(0);
    intrinsics->add_k(1);

    auto projection = _msg.mutable_projection();
    projection->add_p(1);
    projection->add_p(0);
    projection->add_p(0);
    projection->add_p(0);
    projection->add_p(0);
    projection->add_p(1);
    projection->add_p(0);
    projection->add_p(0);
    projection->add_p(0);
    projection->add_p(0);
    projection->add_p(1);
    projection->add_p(0);

    _msg.add_rectification_matrix(1);
    _msg.add_rectification_matrix(0);
    _msg.add_rectification_matrix(0);
    _msg.add_rectification_matrix(0);
    _msg.add_rectification_matrix(1);
    _msg.add_rectification_matrix(0);
    _msg.add_rectification_matrix(0);
    _msg.add_rectification_matrix(0);
    _msg.add_rectification_matrix(1);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::CameraInfo &_msg)
  {
    ignition::msgs::CameraInfo expected_msg;
    createTestMsg(expected_msg);

    ASSERT_TRUE(expected_msg.has_header());
    ASSERT_TRUE(_msg.has_header());

    compareTestMsg(_msg.header());
    EXPECT_EQ(expected_msg.width(), _msg.width());
    EXPECT_EQ(expected_msg.height(), _msg.height());

    ASSERT_TRUE(expected_msg.has_distortion());
    ASSERT_TRUE(_msg.has_distortion());

    auto distortion = _msg.distortion();
    auto expected_distortion = expected_msg.distortion();
    EXPECT_EQ(expected_distortion.model(), distortion.model());
    ASSERT_EQ(expected_distortion.k_size(), distortion.k_size());
    for (auto i = 0; i < expected_distortion.k_size(); ++i)
      EXPECT_EQ(expected_distortion.k(i), distortion.k(i));

    ASSERT_TRUE(expected_msg.has_intrinsics());
    ASSERT_TRUE(_msg.has_intrinsics());

    auto intrinsics = _msg.intrinsics();
    auto expected_intrinsics = expected_msg.intrinsics();
    ASSERT_EQ(expected_intrinsics.k_size(), intrinsics.k_size());
    for (auto i = 0; i < expected_intrinsics.k_size(); ++i)
      EXPECT_EQ(expected_intrinsics.k(i), intrinsics.k(i));

    ASSERT_TRUE(expected_msg.has_projection());
    ASSERT_TRUE(_msg.has_projection());

    auto projection = _msg.projection();
    auto expected_projection = expected_msg.projection();
    ASSERT_EQ(expected_projection.p_size(), projection.p_size());
    for (auto i = 0; i < expected_projection.p_size(); ++i)
      EXPECT_EQ(expected_projection.p(i), projection.p(i));

    ASSERT_EQ(expected_msg.rectification_matrix_size(), _msg.rectification_matrix_size());
    for (auto i = 0; i < expected_msg.rectification_matrix_size(); ++i)
      EXPECT_EQ(expected_msg.rectification_matrix(i), _msg.rectification_matrix(i));
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::FluidPressure &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.set_pressure(0.123);
    _msg.set_variance(0.456);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::FluidPressure &_msg)
  {
    ignition::msgs::FluidPressure expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());
    EXPECT_FLOAT_EQ(expected_msg.pressure(), _msg.pressure());
    EXPECT_FLOAT_EQ(expected_msg.variance(), _msg.variance());
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
    compareTestMsg(_msg.header());
    compareTestMsg(_msg.orientation());
    compareTestMsg(_msg.angular_velocity());
    compareTestMsg(_msg.linear_acceleration());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Axis &_msg)
  {
    _msg.set_position(1.0);
    _msg.set_velocity(2.0);
    _msg.set_force(3.0);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Axis &_msg)
  {
    ignition::msgs::Axis expected_msg;
    createTestMsg(expected_msg);

    EXPECT_DOUBLE_EQ(expected_msg.position(), _msg.position());
    EXPECT_DOUBLE_EQ(expected_msg.velocity(), _msg.velocity());
    EXPECT_DOUBLE_EQ(expected_msg.force(),    _msg.force());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Model &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);
    _msg.mutable_header()->CopyFrom(header_msg);

    for (auto i = 0; i < 3; ++i)
    {
      auto newJoint = _msg.add_joint();
      newJoint->set_name("joint_" + std::to_string(i));

      ignition::msgs::Axis axis_msg;
      createTestMsg(axis_msg);
      newJoint->mutable_axis1()->CopyFrom(axis_msg);
    }
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Model &_msg)
  {
    ignition::msgs::Model expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());

    ASSERT_EQ(expected_msg.joint_size(), _msg.joint_size());
    for (auto i = 0; i < _msg.joint_size(); ++i)
    {
      EXPECT_EQ(expected_msg.joint(i).name(), _msg.joint(i).name());
      compareTestMsg(_msg.joint(i).axis1());
    }
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
    compareTestMsg(_msg.header());
    compareTestMsg(_msg.field_tesla());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Actuators &_msg)
  {
    ignition::msgs::Header header_msg;

    createTestMsg(header_msg);
    _msg.mutable_header()->CopyFrom(header_msg);

    for (int i = 0u; i < 5; ++i)
    {
      _msg.add_position(i);
      _msg.add_velocity(i);
      _msg.add_normalized(i);
    }
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Actuators &_msg)
  {
    ignition::msgs::Actuators expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.header());

    for (int i = 0; i < expected_msg.position_size(); ++i)
    {
      EXPECT_EQ(expected_msg.position(i),   _msg.position(i));
      EXPECT_EQ(expected_msg.velocity(i),   _msg.velocity(i));
      EXPECT_EQ(expected_msg.normalized(i), _msg.normalized(i));
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::OccupancyGrid &_msg)
  {
    ignition::msgs::Header header_msg;
    ignition::msgs::Pose pose_msg;

    createTestMsg(header_msg);
    createTestMsg(pose_msg);

    _msg.mutable_header()->CopyFrom(header_msg);

    _msg.mutable_info()->mutable_map_load_time()->set_sec(100);
    _msg.mutable_info()->mutable_map_load_time()->set_nsec(200);
    _msg.mutable_info()->set_resolution(0.05);
    _msg.mutable_info()->set_width(10);
    _msg.mutable_info()->set_height(20);

    _msg.mutable_info()->mutable_origin()->CopyFrom(pose_msg);

    std::vector<int8_t> data(_msg.info().height() * _msg.info().width(), 1);
    _msg.set_data(&data[0], data.size());
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::OccupancyGrid &_msg)
  {
    compareTestMsg(_msg.header());

    EXPECT_EQ(100, _msg.info().map_load_time().sec());
    EXPECT_EQ(200, _msg.info().map_load_time().nsec());
    EXPECT_FLOAT_EQ(0.05, _msg.info().resolution());
    EXPECT_EQ(10u, _msg.info().width());
    EXPECT_EQ(20u, _msg.info().height());
    compareTestMsg(_msg.info().origin());

    EXPECT_EQ(20u * 10u, _msg.data().size());
    EXPECT_EQ('\1', _msg.data()[0]);
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Odometry &_msg)
  {
    ignition::msgs::Header header_msg;
    ignition::msgs::Pose pose_msg;
    ignition::msgs::Twist twist_msg;

    createTestMsg(header_msg);
    createTestMsg(pose_msg);
    createTestMsg(twist_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.mutable_pose()->CopyFrom(pose_msg);
    _msg.mutable_twist()->CopyFrom(twist_msg);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Odometry &_msg)
  {
    compareTestMsg(_msg.header());
    compareTestMsg(_msg.pose());
    compareTestMsg(_msg.twist());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::PointCloudPacked &_msg)
  {
    ignition::msgs::Header header_msg;

    createTestMsg(header_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    ignition::msgs::PointCloudPacked::Field *field = _msg.add_field();
    field->set_name("x");
    field->set_offset(0);
    field->set_datatype(ignition::msgs::PointCloudPacked::Field::FLOAT32);
    field->set_count(1);

    uint32_t height = 4;
    uint32_t width = 2;

    _msg.set_height(height);
    _msg.set_width(width);
    _msg.set_is_bigendian(false);
    _msg.set_point_step(4);
    _msg.set_row_step(4 * width);
    _msg.set_is_dense(true);

    std::string *msgBuffer = _msg.mutable_data();
    msgBuffer->resize(_msg.row_step() * _msg.height());
    char *msgBufferIndex = msgBuffer->data();

    for (uint32_t j = 0; j < height; ++j)
    {
      for (uint32_t i = 0; i < width; ++i)
      {
        *reinterpret_cast<float*>(msgBufferIndex + _msg.field(0).offset()) =
          j * width + i;
        msgBufferIndex += _msg.point_step();
      }
    }
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::PointCloudPacked &_msg)
  {
    compareTestMsg(_msg.header());

    uint32_t height = 4;
    uint32_t width = 2;

    EXPECT_EQ(height, _msg.height());
    EXPECT_EQ(width, _msg.width());
    EXPECT_FALSE(_msg.is_bigendian());
    EXPECT_EQ(4u, _msg.point_step());
    EXPECT_EQ(4U * width, _msg.row_step());
    EXPECT_TRUE(_msg.is_dense());

    char *msgBufferIndex = const_cast<char*>(_msg.data().data());

    for (uint32_t j = 0; j < height; ++j)
    {
      for (uint32_t i = 0; i < width; ++i)
      {
        float *value =
          reinterpret_cast<float*>(msgBufferIndex + _msg.field(0).offset());

        EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
        msgBufferIndex += _msg.point_step();
      }
    }
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::BatteryState &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.mutable_header()->CopyFrom(header_msg);
    _msg.set_voltage(123);
    _msg.set_current(456);
    _msg.set_charge(789);
    _msg.set_capacity(321);
    _msg.set_percentage(654);
    _msg.set_power_supply_status(ignition::msgs::BatteryState::DISCHARGING);
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::BatteryState &_msg)
  {
    ignition::msgs::BatteryState expected_msg;
    createTestMsg(expected_msg);

    ASSERT_TRUE(expected_msg.has_header());
    ASSERT_TRUE(_msg.has_header());

    compareTestMsg(_msg.header());
    EXPECT_EQ(expected_msg.voltage(), _msg.voltage());
    EXPECT_EQ(expected_msg.current(), _msg.current());
    EXPECT_EQ(expected_msg.charge(), _msg.charge());
    EXPECT_EQ(expected_msg.capacity(), _msg.capacity());
    EXPECT_EQ(expected_msg.percentage(), _msg.percentage());
    EXPECT_EQ(expected_msg.power_supply_status(), _msg.power_supply_status());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Marker &_msg)
  {
    ignition::msgs::Header header_msg;
    createTestMsg(header_msg);

    _msg.mutable_header()->CopyFrom(header_msg);

    _msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    _msg.set_ns("foo");
    _msg.set_id(123);
    _msg.set_type(ignition::msgs::Marker::BOX);
    _msg.mutable_lifetime()->set_sec(100);
    _msg.mutable_lifetime()->set_nsec(200);

    createTestMsg(*_msg.mutable_pose());
    createTestMsg(*_msg.mutable_scale());

    createTestMsg(*_msg.mutable_material()->mutable_ambient());
    createTestMsg(*_msg.mutable_material()->mutable_diffuse());
    createTestMsg(*_msg.mutable_material()->mutable_specular());

    createTestMsg(*_msg.add_point());
    
    _msg.set_text("bar");

  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Marker &_msg)
  {
    ignition::msgs::Marker expected_msg;
    createTestMsg(expected_msg);

    ASSERT_TRUE(expected_msg.has_header());
    ASSERT_TRUE(_msg.has_header());

    compareTestMsg(_msg.header());

    EXPECT_EQ(expected_msg.action(), _msg.action());
    EXPECT_EQ(expected_msg.ns(), _msg.ns());
    EXPECT_EQ(expected_msg.id(), _msg.id());
    EXPECT_EQ(expected_msg.type(), _msg.type());
    EXPECT_EQ(expected_msg.lifetime().sec(), _msg.lifetime().sec());
    EXPECT_EQ(expected_msg.lifetime().nsec(), _msg.lifetime().nsec());

    compareTestMsg(_msg.pose());
    compareTestMsg(_msg.scale());
    compareTestMsg(_msg.material().ambient());
    compareTestMsg(_msg.material().diffuse());
    compareTestMsg(_msg.material().specular());

    compareTestMsg(_msg.point(0));
    
    EXPECT_EQ(expected_msg.text(), _msg.text());
  }

  /// \brief Create a message used for testing.
  /// \param[out] _msg The message populated.
  void createTestMsg(ignition::msgs::Marker_V &_msg)
  {
    // Not setting header because the ROS MarkerArray doesn't use it.
    createTestMsg(*_msg.add_marker());
  }

  /// \brief Compare a message with the populated for testing.
  /// \param[in] _msg The message to compare.
  void compareTestMsg(const ignition::msgs::Marker_V &_msg)
  {
    ignition::msgs::Marker_V expected_msg;
    createTestMsg(expected_msg);

    compareTestMsg(_msg.marker(0));
  }
}
}

#endif  // ROS_IGN_BRIDGE__TEST_UTILS_H_
