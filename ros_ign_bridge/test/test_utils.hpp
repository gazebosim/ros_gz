// Copyright 2020 Open Source Robotics Foundation, Inc.
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


#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <gtest/gtest.h>

#include <ignition/msgs.hh>


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
// #include <mav_msgs/msg/Actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_ign_interfaces/msg/entity.hpp>
#include <ros_ign_interfaces/msg/joint_wrench.hpp>
#include <ros_ign_interfaces/msg/contact.hpp>
#include <ros_ign_interfaces/msg/contacts.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <string>
#include <thread>
#include <memory>

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
template<class Rep, class Period>
void waitUntilBoolVar(
  bool & _boolVar,
  const std::chrono::duration<Rep, Period> & _sleepEach,
  const int _retries)
{
  int i = 0;
  while (!_boolVar && i < _retries) {
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
template<class Rep, class Period>
void waitUntilBoolVarAndSpin(
  std::shared_ptr<rclcpp::Node> & node,
  bool & _boolVar,
  const std::chrono::duration<Rep, Period> & _sleepEach,
  const int _retries)
{
  int i = 0;
  while (!_boolVar && i < _retries) {
    ++i;
    std::this_thread::sleep_for(_sleepEach);
    rclcpp::spin_some(node);
  }
}

//////////////////////////////////////////////////
/// ROS test utils
//////////////////////////////////////////////////

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Bool & _msg)
{
  _msg.data = true;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Bool> & _msg)
{
  std_msgs::msg::Bool expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

/// \brief Compare a message with the populated for testing. Noop for Empty
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Empty> &)
{
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Float32 & _msg)
{
  _msg.data = 1.5;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float32> & _msg)
{
  std_msgs::msg::Float32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Float64 & _msg)
{
  _msg.data = 1.5;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float64> & _msg)
{
  std_msgs::msg::Float64 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}


/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::UInt32 & _msg)
{
  _msg.data = 1;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::UInt32> & _msg)
{
  std_msgs::msg::UInt32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Header & _msg)
{
  // _msg.seq        = 1;
  _msg.stamp.sec = 2;
  _msg.stamp.nanosec = 3;
  _msg.frame_id = "frame_id_value";
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Header> & _msg)
{
  std_msgs::msg::Header expected_msg;
  createTestMsg(expected_msg);

  // EXPECT_GE(expected_msg.seq,        0u);
  EXPECT_EQ(expected_msg.stamp.sec, _msg->stamp.sec);
  EXPECT_EQ(expected_msg.stamp.nanosec, _msg->stamp.nanosec);
  EXPECT_EQ(expected_msg.frame_id, _msg->frame_id);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std_msgs::msg::Header & _msg)
{
  std_msgs::msg::Header expected_msg;
  createTestMsg(expected_msg);

  // EXPECT_GE(expected_msg.seq,        0u);
  EXPECT_EQ(expected_msg.stamp.sec, _msg.stamp.sec);
  EXPECT_EQ(expected_msg.stamp.nanosec, _msg.stamp.nanosec);
  EXPECT_EQ(expected_msg.frame_id, _msg.frame_id);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::String & _msg)
{
  _msg.data = "string";
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::String> & _msg)
{
  std_msgs::msg::String expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Quaternion & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
  _msg.w = 4;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::Quaternion & _msg)
{
  geometry_msgs::msg::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg.x);
  EXPECT_EQ(expected_msg.y, _msg.y);
  EXPECT_EQ(expected_msg.z, _msg.z);
  EXPECT_EQ(expected_msg.w, _msg.w);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Quaternion> & _msg)
{
  geometry_msgs::msg::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
  EXPECT_EQ(expected_msg.w, _msg->w);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Vector3 & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Vector3> & _msg)
{
  geometry_msgs::msg::Vector3 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(rosgraph_msgs::msg::Clock & _msg)
{
  _msg.clock.sec = 1;
  _msg.clock.nanosec = 2;
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Point & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<rosgraph_msgs::msg::Clock> & _msg)
{
  rosgraph_msgs::msg::Clock expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.clock.sec, _msg->clock.sec);
  EXPECT_EQ(expected_msg.clock.nanosec, _msg->clock.nanosec);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Point> & _msg)
{
  geometry_msgs::msg::Point expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::Point & _msg)
{
  geometry_msgs::msg::Point expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg.x);
  EXPECT_EQ(expected_msg.y, _msg.y);
  EXPECT_EQ(expected_msg.z, _msg.z);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::Pose & _msg)
{
  compareTestMsg(_msg.position);
  compareTestMsg(_msg.orientation);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Pose & _msg)
{
  createTestMsg(_msg.position);
  createTestMsg(_msg.orientation);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Pose> & _msg)
{
  compareTestMsg(_msg->position);
  compareTestMsg(_msg->orientation);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::PoseStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseStamped> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->pose);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::PoseStamped & _msg)
{
  compareTestMsg(_msg.header);
  compareTestMsg(_msg.pose);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Transform & _msg)
{
  createTestMsg(_msg.translation);
  createTestMsg(_msg.rotation);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Transform> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->translation));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Quaternion>(_msg->rotation));
}
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::TransformStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.transform);
  _msg.child_frame_id = "child_frame_id_value";
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TransformStamped> & _msg)
{
  geometry_msgs::msg::TransformStamped expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Transform>(_msg->transform));
  EXPECT_EQ(expected_msg.child_frame_id, _msg->child_frame_id);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(tf2_msgs::msg::TFMessage & _msg)
{
  geometry_msgs::msg::TransformStamped tf;
  createTestMsg(tf);
  _msg.transforms.push_back(tf);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const tf2_msgs::msg::TFMessage & _msg)
{
  tf2_msgs::msg::TFMessage expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<geometry_msgs::msg::TransformStamped>(_msg.transforms[0]));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Twist & _msg)
{
  createTestMsg(_msg.linear);
  createTestMsg(_msg.angular);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Twist> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->linear));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->angular));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Wrench & _msg)
{
  createTestMsg(_msg.force);
  createTestMsg(_msg.torque);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Wrench> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->force));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->torque));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_ign_interfaces::msg::JointWrench & _msg)
{
  createTestMsg(_msg.header);
  _msg.body_1_name.data = "body1";
  _msg.body_2_name.data = "body2";
  _msg.body_1_id.data = 1;
  _msg.body_2_id.data = 2;
  createTestMsg(_msg.body_1_wrench);
  createTestMsg(_msg.body_2_wrench);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::JointWrench> & _msg)
{
  ros_ign_interfaces::msg::JointWrench expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.body_1_name, _msg->body_1_name);
  EXPECT_EQ(expected_msg.body_2_name, _msg->body_2_name);
  EXPECT_EQ(expected_msg.body_1_id, _msg->body_1_id);
  EXPECT_EQ(expected_msg.body_2_id, _msg->body_2_id);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->body_1_wrench));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->body_2_wrench));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_ign_interfaces::msg::Entity & _msg)
{
  _msg.id = 1;
  _msg.name = "entity";
  _msg.type = ros_ign_interfaces::msg::Entity::VISUAL;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Entity> & _msg)
{
  ros_ign_interfaces::msg::Entity expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.id, _msg->id);
  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.type, _msg->type);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_ign_interfaces::msg::Contact & _msg)
{
  createTestMsg(_msg.collision1);
  createTestMsg(_msg.collision2);

  geometry_msgs::msg::Vector3 vector_msg;
  createTestMsg(vector_msg);

  ros_ign_interfaces::msg::JointWrench joint_wrench_msg;
  createTestMsg(joint_wrench_msg);

  for (int i = 0; i < 10; i++) {
    _msg.depths.push_back(i);
    _msg.positions.push_back(vector_msg);
    _msg.normals.push_back(vector_msg);
    _msg.wrenches.push_back(joint_wrench_msg);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Contact> & _msg)
{
  ros_ign_interfaces::msg::Contact expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<ros_ign_interfaces::msg::Entity>(_msg->collision1));
  compareTestMsg(std::make_shared<ros_ign_interfaces::msg::Entity>(_msg->collision2));
  EXPECT_EQ(expected_msg.depths.size(), _msg->depths.size());
  EXPECT_EQ(expected_msg.positions.size(), _msg->positions.size());
  EXPECT_EQ(expected_msg.normals.size(), _msg->normals.size());
  EXPECT_EQ(expected_msg.wrenches.size(), _msg->wrenches.size());
  for (size_t i = 0; i < _msg->depths.size(); i++) {
    EXPECT_EQ(expected_msg.depths.at(i), _msg->depths.at(i));
    compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->positions.at(i)));
    compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->normals.at(i)));
    compareTestMsg(std::make_shared<ros_ign_interfaces::msg::JointWrench>(_msg->wrenches.at(i)));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_ign_interfaces::msg::Contacts & _msg)
{
  createTestMsg(_msg.header);

  ros_ign_interfaces::msg::Contact contact_msg;
  createTestMsg(contact_msg);

  for (int i = 0; i < 10; i++) {
    _msg.contacts.push_back(contact_msg);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Contacts> & _msg)
{
  ros_ign_interfaces::msg::Contacts expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.contacts.size(), _msg->contacts.size());
  for (size_t i = 0; i < _msg->contacts.size(); i++) {
    compareTestMsg(std::make_shared<ros_ign_interfaces::msg::Contact>(_msg->contacts.at(i)));
  }
}

// /// \brief Create a message used for testing.
// /// \param[out] _msg The message populated.
// void createTestMsg(mav_msgs::msg::Actuators> &_msg)
// {
//   createTestMsg(_msg.header);
//   for (auto i = 0u; i < 5; ++i)
//   {
//     _msg.angles.push_back(i);
//     _msg.angular_velocities.push_back(i);
//     _msg.normalized.push_back(i);
//   }
// }
//
// /// \brief Compare a message with the populated for testing.
// /// \param[in] _msg The message to compare.
// void compareTestMsg(const std::shared_ptr<mav_msgs::msg::Actuators> &_msg)
// {
//   mav_msgs::msg::Actuators expected_msg;
//   createTestMsg(expected_msg);
//
//   compareTestMsg(_msg.header);
//
//   ASSERT_EQ(expected_msg.angles.size(), _msg.angles.size());
//   ASSERT_EQ(expected_msg.angular_velocities.size(),
//             _msg.angular_velocities.size());
//   ASSERT_EQ(expected_msg.normalized.size(), _msg.normalized.size());
//
//   for (auto i = 0u; i < _msg.angles.size(); ++i)
//   {
//     EXPECT_EQ(expected_msg.angles[i], _msg.angles[i]);
//     EXPECT_EQ(expected_msg.angular_velocities[i], _msg.angular_velocities[i]);
//     EXPECT_EQ(expected_msg.normalized[i], _msg.normalized[i]);
//   }
// }

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(nav_msgs::msg::Odometry & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose.pose);
  createTestMsg(_msg.twist.twist);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<nav_msgs::msg::Odometry> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->pose.pose);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Twist>(_msg->twist.twist));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::Image & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.width = 320;
  _msg.height = 240;
  _msg.encoding = "rgb8";
  _msg.is_bigendian = false;
  _msg.step = _msg.width * 3;
  _msg.data.resize(_msg.height * _msg.step, '1');
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Image> & _msg)
{
  sensor_msgs::msg::Image expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.width, _msg->width);
  EXPECT_EQ(expected_msg.height, _msg->height);
  EXPECT_EQ(expected_msg.encoding, _msg->encoding);
  EXPECT_EQ(expected_msg.is_bigendian, _msg->is_bigendian);
  EXPECT_EQ(expected_msg.step, _msg->step);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::CameraInfo & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.width = 320;
  _msg.height = 240;
  _msg.distortion_model = "plumb_bob";
  _msg.d.resize(5);
  _msg.d[0] = 1;
  _msg.d[1] = 2;
  _msg.d[2] = 3;
  _msg.d[3] = 4;
  _msg.d[4] = 5;

  _msg.k[0] = 1;
  _msg.k[1] = 0;
  _msg.k[2] = 0;
  _msg.k[3] = 0;
  _msg.k[4] = 1;
  _msg.k[5] = 0;
  _msg.k[6] = 0;
  _msg.k[7] = 0;
  _msg.k[8] = 1;

  _msg.r[0] = 1;
  _msg.r[1] = 0;
  _msg.r[2] = 0;
  _msg.r[3] = 0;
  _msg.r[4] = 1;
  _msg.r[5] = 0;
  _msg.r[6] = 0;
  _msg.r[7] = 0;
  _msg.r[8] = 1;

  _msg.p[0] = 1;
  _msg.p[1] = 0;
  _msg.p[2] = 0;
  _msg.p[3] = 0;
  _msg.p[4] = 0;
  _msg.p[5] = 1;
  _msg.p[6] = 0;
  _msg.p[7] = 0;
  _msg.p[8] = 0;
  _msg.p[9] = 0;
  _msg.p[10] = 1;
  _msg.p[11] = 0;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::CameraInfo> & _msg)
{
  sensor_msgs::msg::CameraInfo expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.width, _msg->width);
  EXPECT_EQ(expected_msg.height, _msg->height);
  EXPECT_EQ(expected_msg.distortion_model, _msg->distortion_model);

  for (auto i = 0; i < 12; ++i) {
    EXPECT_EQ(expected_msg.p[i], _msg->p[i]);

    if (i > 8) {
      continue;
    }

    EXPECT_EQ(expected_msg.k[i], _msg->k[i]);
    EXPECT_EQ(expected_msg.r[i], _msg->r[i]);

    if (i > 4) {
      continue;
    }

    EXPECT_EQ(expected_msg.d[i], _msg->d[i]);
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::FluidPressure & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.fluid_pressure = 0.123;
  _msg.variance = 0.456;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::FluidPressure> & _msg)
{
  sensor_msgs::msg::FluidPressure expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_FLOAT_EQ(expected_msg.fluid_pressure, _msg->fluid_pressure);
  EXPECT_FLOAT_EQ(expected_msg.variance, _msg->variance);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::Imu & _msg)
{
  std_msgs::msg::Header header_msg;
  geometry_msgs::msg::Quaternion quaternion_msg;
  geometry_msgs::msg::Vector3 vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(quaternion_msg);
  createTestMsg(vector3_msg);

  _msg.header = header_msg;
  _msg.orientation = quaternion_msg;
  _msg.orientation_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  _msg.angular_velocity = vector3_msg;
  _msg.angular_velocity_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  _msg.linear_acceleration = vector3_msg;
  _msg.linear_acceleration_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Imu> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->orientation);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->angular_velocity));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->linear_acceleration));

  for (auto i = 0; i < 9; ++i) {
    EXPECT_FLOAT_EQ(i + 1, _msg->orientation_covariance[i]);
    EXPECT_FLOAT_EQ(i + 1, _msg->angular_velocity_covariance[i]);
    EXPECT_FLOAT_EQ(i + 1, _msg->linear_acceleration_covariance[i]);
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::JointState & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.name = {"joint_0", "joint_1", "joint_2"};
  _msg.position = {1, 1, 1};
  _msg.velocity = {2, 2, 2};
  _msg.effort = {3, 3, 3};
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::JointState> & _msg)
{
  sensor_msgs::msg::JointState expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  ASSERT_EQ(expected_msg.name.size(), _msg->name.size());
  ASSERT_EQ(expected_msg.position.size(), _msg->position.size());
  ASSERT_EQ(expected_msg.velocity.size(), _msg->velocity.size());
  ASSERT_EQ(expected_msg.effort.size(), _msg->effort.size());

  for (auto i = 0u; i < _msg->position.size(); ++i) {
    EXPECT_EQ(expected_msg.name[i], _msg->name[i]);
    EXPECT_FLOAT_EQ(expected_msg.position[i], _msg->position[i]);
    EXPECT_FLOAT_EQ(expected_msg.velocity[i], _msg->velocity[i]);
    EXPECT_FLOAT_EQ(expected_msg.effort[i], _msg->effort[i]);
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::LaserScan & _msg)
{
  const unsigned int num_readings = 100u;
  const double laser_frequency = 40;

  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.angle_min = -1.57;
  _msg.angle_max = 1.57;
  _msg.angle_increment = 3.14 / num_readings;
  _msg.time_increment = (1 / laser_frequency) / (num_readings);
  _msg.scan_time = 0;
  _msg.range_min = 1;
  _msg.range_max = 2;
  _msg.ranges.resize(num_readings, 0);
  _msg.intensities.resize(num_readings, 1);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::LaserScan> & _msg)
{
  sensor_msgs::msg::LaserScan expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_FLOAT_EQ(expected_msg.angle_min, _msg->angle_min);
  EXPECT_FLOAT_EQ(expected_msg.angle_max, _msg->angle_max);
  EXPECT_FLOAT_EQ(expected_msg.angle_increment, _msg->angle_increment);
  EXPECT_FLOAT_EQ(0.00025000001, _msg->time_increment);
  EXPECT_FLOAT_EQ(0, _msg->scan_time);
  EXPECT_FLOAT_EQ(expected_msg.range_min, _msg->range_min);
  EXPECT_FLOAT_EQ(expected_msg.range_max, _msg->range_max);

  const unsigned int num_readings =
    (_msg->angle_max - _msg->angle_min) / _msg->angle_increment;
  for (auto i = 0u; i < num_readings; ++i) {
    EXPECT_FLOAT_EQ(expected_msg.ranges[i], _msg->ranges[i]);
    EXPECT_FLOAT_EQ(expected_msg.intensities[i], _msg->intensities[i]);
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::MagneticField & _msg)
{
  std_msgs::msg::Header header_msg;
  geometry_msgs::msg::Vector3 vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(vector3_msg);

  _msg.header = header_msg;
  _msg.magnetic_field = vector3_msg;
  _msg.magnetic_field_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::MagneticField> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->magnetic_field));

  for (auto i = 0u; i < 9; ++i) {
    EXPECT_FLOAT_EQ(i + 1, _msg->magnetic_field_covariance[i]);
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::PointCloud2 & _msg)
{
  createTestMsg(_msg.header);

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
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
  uint8_t * msgBufferIndex = _msg.data.data();

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      *reinterpret_cast<float *>(msgBufferIndex + _msg.fields[0].offset) =
        j * width + i;
      msgBufferIndex += _msg.point_step;
    }
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::PointCloud2> & _msg)
{
  compareTestMsg(_msg->header);

  uint32_t height = 4;
  uint32_t width = 2;

  EXPECT_EQ(height, _msg->height);
  EXPECT_EQ(width, _msg->width);
  EXPECT_FALSE(_msg->is_bigendian);
  EXPECT_EQ(4u, _msg->point_step);
  EXPECT_EQ(4U * width, _msg->row_step);
  EXPECT_TRUE(_msg->is_dense);

  unsigned char * msgBufferIndex =
    const_cast<unsigned char *>(_msg->data.data());

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      float * value =
        reinterpret_cast<float *>(msgBufferIndex + _msg->fields[0].offset);

      EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
      msgBufferIndex += _msg->point_step;
    }
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::BatteryState & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.voltage = 123;
  _msg.current = 456;
  _msg.charge = 789;
  _msg.capacity = 321;
  _msg.percentage = 654;
  _msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::BatteryState> & _msg)
{
  sensor_msgs::msg::BatteryState expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.voltage, _msg->voltage);
  EXPECT_EQ(expected_msg.current, _msg->current);
  EXPECT_EQ(expected_msg.charge, _msg->charge);
  EXPECT_EQ(expected_msg.capacity, _msg->capacity);
  EXPECT_EQ(expected_msg.percentage, _msg->percentage);
  EXPECT_EQ(expected_msg.power_supply_status, _msg->power_supply_status);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(trajectory_msgs::msg::JointTrajectoryPoint & _msg)
{
  const auto number_of_joints = 7;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.positions.push_back(1.1 * i);
    _msg.velocities.push_back(2.2 * i);
    _msg.accelerations.push_back(3.3 * i);
    _msg.effort.push_back(4.4 * i);
  }
  _msg.time_from_start.sec = 12345;
  _msg.time_from_start.nanosec = 67890;
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> & _msg)
{
  trajectory_msgs::msg::JointTrajectoryPoint expected_msg;
  createTestMsg(expected_msg);

  for (auto i = 0u; i < _msg->positions.size(); ++i) {
    EXPECT_EQ(expected_msg.positions[i], _msg->positions[i]);
  }

  for (auto i = 0u; i < _msg->velocities.size(); ++i) {
    EXPECT_EQ(expected_msg.velocities[i], _msg->velocities[i]);
  }

  for (auto i = 0u; i < _msg->accelerations.size(); ++i) {
    EXPECT_EQ(expected_msg.accelerations[i], _msg->accelerations[i]);
  }

  for (auto i = 0u; i < _msg->effort.size(); ++i) {
    EXPECT_EQ(expected_msg.effort[i], _msg->effort[i]);
  }

  EXPECT_EQ(expected_msg.time_from_start.sec, _msg->time_from_start.sec);
  EXPECT_EQ(expected_msg.time_from_start.nanosec, _msg->time_from_start.nanosec);
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(trajectory_msgs::msg::JointTrajectory & _msg)
{
  const auto number_of_joints = 7;
  const auto number_of_trajectory_points = 10;

  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.joint_names.push_back("joint_" + std::to_string(i));
  }

  for (auto j = 0; j < number_of_trajectory_points; ++j) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    createTestMsg(point);
    _msg.points.push_back(point);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & _msg)
{
  trajectory_msgs::msg::JointTrajectory expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  for (auto i = 0u; i < _msg->joint_names.size(); ++i) {
    EXPECT_EQ(expected_msg.joint_names[i], _msg->joint_names[i]);
  }

  for (auto i = 0u; i < _msg->points.size(); ++i) {
    compareTestMsg(std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>(_msg->points[i]));
  }
}

//////////////////////////////////////////////////
/// Ignition::msgs test utils
//////////////////////////////////////////////////

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Boolean & _msg)
{
  _msg.set_data(true);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Boolean> & _msg)
{
  ignition::msgs::Boolean expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data(), _msg->data());
}

/// \brief Compare a message with the populated for testing. Noop for Empty
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Empty> &)
{
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Float & _msg)
{
  _msg.set_data(1.5);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Float> & _msg)
{
  ignition::msgs::Float expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data(), _msg->data());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Double & _msg)
{
  _msg.set_data(1.5);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Double> & _msg)
{
  ignition::msgs::Double expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.data(), _msg->data());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::UInt32 & _msg)
{
  _msg.set_data(1);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::UInt32> & _msg)
{
  ignition::msgs::UInt32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.data(), _msg->data());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Header & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::Header> & _msg)
{
  // TODO(anyone): Review this
  ignition::msgs::Header expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.stamp().sec(), _msg->stamp().sec());
  EXPECT_EQ(expected_msg.stamp().nsec(), _msg->stamp().nsec());
  // EXPECT_GE(_msg->data_size(), 2);
  // EXPECT_EQ(expected_msg.data(0).key(), "seq");
  // EXPECT_EQ(1, _msg->data(0).value_size());
  // std::string value = _msg->data(0).value(0);
  // try {
  //   uint32_t ul = std::stoul(value, nullptr);
  //   EXPECT_GE(ul, 0u);
  // } catch (std::exception & e) {
  //   FAIL();
  // }
  // EXPECT_EQ(expected_msg.data(1).key(), _msg->data(1).key());
  // EXPECT_EQ(1, _msg->data(1).value_size());
  // EXPECT_EQ(expected_msg.data(1).value(0), _msg->data(1).value(0));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Clock & _msg)
{
  _msg.mutable_sim()->set_sec(1);
  _msg.mutable_sim()->set_nsec(2);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Clock> & _msg)
{
  ignition::msgs::Clock expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.sim().sec(), _msg->sim().sec());
  EXPECT_EQ(expected_msg.sim().nsec(), _msg->sim().nsec());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::StringMsg & _msg)
{
  _msg.set_data("string");
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::StringMsg> & _msg)
{
  ignition::msgs::StringMsg expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data(), _msg->data());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Quaternion & _msg)
{
  _msg.set_x(1.0);
  _msg.set_y(2.0);
  _msg.set_z(3.0);
  _msg.set_w(4.0);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Quaternion> & _msg)
{
  ignition::msgs::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x(), _msg->x());
  EXPECT_EQ(expected_msg.y(), _msg->y());
  EXPECT_EQ(expected_msg.z(), _msg->z());
  EXPECT_EQ(expected_msg.w(), _msg->w());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Vector3d & _msg)
{
  _msg.set_x(1.0);
  _msg.set_y(2.0);
  _msg.set_z(3.0);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Vector3d> & _msg)
{
  ignition::msgs::Vector3d expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x(), _msg->x());
  EXPECT_EQ(expected_msg.y(), _msg->y());
  EXPECT_EQ(expected_msg.z(), _msg->z());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Pose & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::Pose> & _msg)
{
  if (_msg->header().data_size() > 0) {
    compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));

    ignition::msgs::Pose expected_msg;
    createTestMsg(expected_msg);

    if (_msg->header().data_size() > 2) {
      // child_frame_id
      ASSERT_EQ(3, expected_msg.header().data_size());
      ASSERT_EQ(3, _msg->header().data_size());
      EXPECT_EQ(
        expected_msg.header().data(2).key(),
        _msg->header().data(2).key());
      EXPECT_EQ(1, _msg->header().data(2).value_size());
      EXPECT_EQ(
        expected_msg.header().data(2).value(0),
        _msg->header().data(2).value(0));
    }
  }

  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->position()));
  compareTestMsg(std::make_shared<ignition::msgs::Quaternion>(_msg->orientation()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Pose_V & _msg)
{
  createTestMsg(*(_msg.mutable_header()));
  createTestMsg(*(_msg.add_pose()));
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Pose_V> & _msg)
{
  ignition::msgs::Pose_V expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<ignition::msgs::Pose>(_msg->pose(0)));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Twist & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::Twist> & _msg)
{
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->linear()));
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->angular()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Wrench & _msg)
{
  ignition::msgs::Header header_msg;
  ignition::msgs::Vector3d force_msg;
  ignition::msgs::Vector3d torque_msg;

  createTestMsg(header_msg);
  createTestMsg(force_msg);
  createTestMsg(torque_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_force()->CopyFrom(force_msg);
  _msg.mutable_torque()->CopyFrom(torque_msg);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Wrench> & _msg)
{
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->force()));
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->torque()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::JointWrench & _msg)
{
  ignition::msgs::Header header_msg;
  ignition::msgs::Wrench body_1_wrench_msg;
  ignition::msgs::Wrench body_2_wrench_msg;

  createTestMsg(header_msg);
  createTestMsg(body_1_wrench_msg);
  createTestMsg(body_2_wrench_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_body_1_name("body1");
  _msg.set_body_2_name("body2");
  _msg.set_body_1_id(1);
  _msg.set_body_2_id(2);
  _msg.mutable_body_1_wrench()->CopyFrom(body_1_wrench_msg);
  _msg.mutable_body_2_wrench()->CopyFrom(body_2_wrench_msg);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::JointWrench> & _msg)
{
  ignition::msgs::JointWrench expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.body_1_name(), _msg->body_1_name());
  EXPECT_EQ(expected_msg.body_2_name(), _msg->body_2_name());
  EXPECT_EQ(expected_msg.body_1_id(), _msg->body_1_id());
  EXPECT_EQ(expected_msg.body_2_id(), _msg->body_2_id());
  compareTestMsg(std::make_shared<ignition::msgs::Wrench>(_msg->body_1_wrench()));
  compareTestMsg(std::make_shared<ignition::msgs::Wrench>(_msg->body_2_wrench()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Entity & _msg)
{
  _msg.set_id(1);
  _msg.set_name("entity");
  _msg.set_type(ignition::msgs::Entity::VISUAL);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Entity> & _msg)
{
  ignition::msgs::Entity expected_msg;
  createTestMsg(expected_msg);
  EXPECT_EQ(expected_msg.id(), _msg->id());
  EXPECT_EQ(expected_msg.name(), _msg->name());
  EXPECT_EQ(expected_msg.type(), _msg->type());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Contact & _msg)
{
  ignition::msgs::Entity collision1;
  ignition::msgs::Entity collision2;
  ignition::msgs::Vector3d position_msg;
  ignition::msgs::Vector3d normal_msg;
  ignition::msgs::JointWrench wrench_msg;

  createTestMsg(collision1);
  createTestMsg(collision2);
  createTestMsg(position_msg);
  createTestMsg(normal_msg);
  createTestMsg(wrench_msg);

  _msg.clear_position();
  _msg.clear_normal();
  _msg.clear_wrench();

  for (int i = 0; i < 10; i++) {
    _msg.add_depth(i);
    auto position = _msg.add_position();
    position->set_x(position_msg.x());
    position->set_y(position_msg.y());
    position->set_z(position_msg.z());
    auto normal = _msg.add_normal();
    normal->set_x(normal_msg.x());
    normal->set_y(normal_msg.y());
    normal->set_z(normal_msg.z());
    auto wrench = _msg.add_wrench();
    wrench->mutable_header()->CopyFrom(wrench_msg.header());
    wrench->set_body_1_name(wrench_msg.body_1_name());
    wrench->set_body_2_name(wrench_msg.body_2_name());
    wrench->set_body_1_id(wrench_msg.body_1_id());
    wrench->set_body_2_id(wrench_msg.body_2_id());
    wrench->mutable_body_1_wrench()->CopyFrom(wrench_msg.body_1_wrench());
    wrench->mutable_body_2_wrench()->CopyFrom(wrench_msg.body_2_wrench());
  }
  _msg.mutable_collision1()->CopyFrom(collision1);
  _msg.mutable_collision2()->CopyFrom(collision2);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Contact> & _msg)
{
  ignition::msgs::Contact expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<ignition::msgs::Entity>(_msg->collision1()));
  compareTestMsg(std::make_shared<ignition::msgs::Entity>(_msg->collision2()));
  EXPECT_EQ(expected_msg.depth_size(), _msg->depth_size());
  EXPECT_EQ(expected_msg.position_size(), _msg->position_size());
  EXPECT_EQ(expected_msg.normal_size(), _msg->normal_size());
  EXPECT_EQ(expected_msg.wrench_size(), _msg->wrench_size());
  for (int i = 0; i < expected_msg.depth_size(); i++) {
    EXPECT_EQ(expected_msg.depth(i), _msg->depth(i));
    compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->position(i)));
    compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->normal(i)));
    compareTestMsg(std::make_shared<ignition::msgs::JointWrench>(_msg->wrench(i)));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Contacts & _msg)
{
  ignition::msgs::Header header_msg;
  ignition::msgs::Contact contact_msg;

  createTestMsg(header_msg);
  createTestMsg(contact_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.clear_contact();
  for (int i = 0; i < 10; i++) {
    auto contact = _msg.add_contact();
    contact->mutable_collision1()->CopyFrom(contact_msg.collision1());
    contact->mutable_collision2()->CopyFrom(contact_msg.collision2());
    contact->mutable_position()->CopyFrom(contact_msg.position());
    contact->mutable_normal()->CopyFrom(contact_msg.normal());
    contact->mutable_wrench()->CopyFrom(contact_msg.wrench());
    contact->mutable_depth()->CopyFrom(contact_msg.depth());
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Contacts> & _msg)
{
  ignition::msgs::Contacts expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.contact_size(), _msg->contact_size());
  for (int i = 0; i < expected_msg.contact_size(); i++) {
    compareTestMsg(std::make_shared<ignition::msgs::Contact>(_msg->contact(i)));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Image & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::Image> & _msg)
{
  ignition::msgs::Image expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.width(), _msg->width());
  EXPECT_EQ(expected_msg.height(), _msg->height());
  EXPECT_EQ(expected_msg.pixel_format_type(), _msg->pixel_format_type());
  EXPECT_EQ(expected_msg.step(), _msg->step());
  EXPECT_EQ(expected_msg.data(), _msg->data());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::CameraInfo & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::CameraInfo> & _msg)
{
  ignition::msgs::CameraInfo expected_msg;
  createTestMsg(expected_msg);

  ASSERT_TRUE(expected_msg.has_header());
  ASSERT_TRUE(_msg->has_header());

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.width(), _msg->width());
  EXPECT_EQ(expected_msg.height(), _msg->height());

  ASSERT_TRUE(expected_msg.has_distortion());
  ASSERT_TRUE(_msg->has_distortion());

  auto distortion = _msg->distortion();
  auto expected_distortion = expected_msg.distortion();
  EXPECT_EQ(expected_distortion.model(), distortion.model());
  ASSERT_EQ(expected_distortion.k_size(), distortion.k_size());
  for (auto i = 0; i < expected_distortion.k_size(); ++i) {
    EXPECT_EQ(expected_distortion.k(i), distortion.k(i));
  }

  ASSERT_TRUE(expected_msg.has_intrinsics());
  ASSERT_TRUE(_msg->has_intrinsics());

  auto intrinsics = _msg->intrinsics();
  auto expected_intrinsics = expected_msg.intrinsics();
  ASSERT_EQ(expected_intrinsics.k_size(), intrinsics.k_size());
  for (auto i = 0; i < expected_intrinsics.k_size(); ++i) {
    EXPECT_EQ(expected_intrinsics.k(i), intrinsics.k(i));
  }

  ASSERT_TRUE(expected_msg.has_projection());
  ASSERT_TRUE(_msg->has_projection());

  auto projection = _msg->projection();
  auto expected_projection = expected_msg.projection();
  ASSERT_EQ(expected_projection.p_size(), projection.p_size());
  for (auto i = 0; i < expected_projection.p_size(); ++i) {
    EXPECT_EQ(expected_projection.p(i), projection.p(i));
  }

  ASSERT_EQ(expected_msg.rectification_matrix_size(), _msg->rectification_matrix_size());
  for (auto i = 0; i < expected_msg.rectification_matrix_size(); ++i) {
    EXPECT_EQ(expected_msg.rectification_matrix(i), _msg->rectification_matrix(i));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::FluidPressure & _msg)
{
  ignition::msgs::Header header_msg;
  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_pressure(0.123);
  _msg.set_variance(0.456);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::FluidPressure> & _msg)
{
  ignition::msgs::FluidPressure expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_FLOAT_EQ(expected_msg.pressure(), _msg->pressure());
  EXPECT_FLOAT_EQ(expected_msg.variance(), _msg->variance());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::IMU & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::IMU> & _msg)
{
  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<ignition::msgs::Quaternion>(_msg->orientation()));
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->angular_velocity()));
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->linear_acceleration()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Axis & _msg)
{
  _msg.set_position(1.0);
  _msg.set_velocity(2.0);
  _msg.set_force(3.0);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Axis> & _msg)
{
  ignition::msgs::Axis expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.position(), _msg->position());
  EXPECT_DOUBLE_EQ(expected_msg.velocity(), _msg->velocity());
  EXPECT_DOUBLE_EQ(expected_msg.force(), _msg->force());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Model & _msg)
{
  ignition::msgs::Header header_msg;
  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (auto i = 0; i < 3; ++i) {
    auto newJoint = _msg.add_joint();
    newJoint->set_name("joint_" + std::to_string(i));

    ignition::msgs::Axis axis_msg;
    createTestMsg(axis_msg);
    newJoint->mutable_axis1()->CopyFrom(axis_msg);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Model> & _msg)
{
  ignition::msgs::Model expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));

  ASSERT_EQ(expected_msg.joint_size(), _msg->joint_size());
  for (auto i = 0; i < _msg->joint_size(); ++i) {
    EXPECT_EQ(expected_msg.joint(i).name(), _msg->joint(i).name());
    compareTestMsg(std::make_shared<ignition::msgs::Axis>(_msg->joint(i).axis1()));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::LaserScan & _msg)
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

  for (auto i = 0u; i < _msg.count(); ++i) {
    _msg.add_ranges(0);
    _msg.add_intensities(1);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::LaserScan> & _msg)
{
  ignition::msgs::LaserScan expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_FLOAT_EQ(expected_msg.angle_min(), _msg->angle_min());
  EXPECT_FLOAT_EQ(expected_msg.angle_max(), _msg->angle_max());
  EXPECT_FLOAT_EQ(expected_msg.angle_step(), _msg->angle_step());
  EXPECT_DOUBLE_EQ(expected_msg.range_min(), _msg->range_min());
  EXPECT_DOUBLE_EQ(expected_msg.range_max(), _msg->range_max());
  EXPECT_EQ(expected_msg.count(), _msg->count());
  EXPECT_DOUBLE_EQ(
    expected_msg.vertical_angle_min(),
    _msg->vertical_angle_min());
  EXPECT_DOUBLE_EQ(
    expected_msg.vertical_angle_max(),
    _msg->vertical_angle_max());
  EXPECT_DOUBLE_EQ(
    expected_msg.vertical_angle_step(),
    _msg->vertical_angle_step());
  EXPECT_EQ(expected_msg.vertical_count(), _msg->vertical_count());
  EXPECT_EQ(expected_msg.ranges_size(), _msg->ranges_size());
  EXPECT_EQ(expected_msg.intensities_size(), _msg->intensities_size());

  for (auto i = 0u; i < _msg->count(); ++i) {
    EXPECT_DOUBLE_EQ(expected_msg.ranges(i), _msg->ranges(i));
    EXPECT_DOUBLE_EQ(expected_msg.intensities(i), _msg->intensities(i));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Magnetometer & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::Magnetometer> & _msg)
{
  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<ignition::msgs::Vector3d>(_msg->field_tesla()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Actuators & _msg)
{
  ignition::msgs::Header header_msg;

  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (int i = 0u; i < 5; ++i) {
    _msg.add_position(i);
    _msg.add_velocity(i);
    _msg.add_normalized(i);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Actuators> & _msg)
{
  ignition::msgs::Actuators expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));

  for (int i = 0; i < expected_msg.position_size(); ++i) {
    EXPECT_EQ(expected_msg.position(i), _msg->position(i));
    EXPECT_EQ(expected_msg.velocity(i), _msg->velocity(i));
    EXPECT_EQ(expected_msg.normalized(i), _msg->normalized(i));
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Odometry & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::Odometry> & _msg)
{
  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<ignition::msgs::Pose>(_msg->pose()));
  compareTestMsg(std::make_shared<ignition::msgs::Twist>(_msg->twist()));
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::PointCloudPacked & _msg)
{
  ignition::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  ignition::msgs::PointCloudPacked::Field * field = _msg.add_field();
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

  std::string * msgBuffer = _msg.mutable_data();
  msgBuffer->resize(_msg.row_step() * _msg.height());
  char * msgBufferIndex = msgBuffer->data();

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      *reinterpret_cast<float *>(msgBufferIndex + _msg.field(0).offset()) =
        j * width + i;
      msgBufferIndex += _msg.point_step();
    }
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::PointCloudPacked> & _msg)
{
  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));

  uint32_t height = 4;
  uint32_t width = 2;

  EXPECT_EQ(height, _msg->height());
  EXPECT_EQ(width, _msg->width());
  EXPECT_FALSE(_msg->is_bigendian());
  EXPECT_EQ(4u, _msg->point_step());
  EXPECT_EQ(4U * width, _msg->row_step());
  EXPECT_TRUE(_msg->is_dense());

  char * msgBufferIndex = const_cast<char *>(_msg->data().data());

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      float * value =
        reinterpret_cast<float *>(msgBufferIndex + _msg->field(0).offset());

      EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
      msgBufferIndex += _msg->point_step();
    }
  }
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::BatteryState & _msg)
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
void compareTestMsg(const std::shared_ptr<ignition::msgs::BatteryState> & _msg)
{
  ignition::msgs::BatteryState expected_msg;
  createTestMsg(expected_msg);

  ASSERT_TRUE(expected_msg.has_header());
  ASSERT_TRUE(_msg->has_header());

  compareTestMsg(std::make_shared<ignition::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.voltage(), _msg->voltage());
  EXPECT_EQ(expected_msg.current(), _msg->current());
  EXPECT_EQ(expected_msg.charge(), _msg->charge());
  EXPECT_EQ(expected_msg.capacity(), _msg->capacity());
  EXPECT_EQ(expected_msg.percentage(), _msg->percentage());
  EXPECT_EQ(expected_msg.power_supply_status(), _msg->power_supply_status());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::JointTrajectoryPoint & _msg)
{
  const auto number_of_joints = 7;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.add_positions(1.1 * i);
    _msg.add_velocities(2.2 * i);
    _msg.add_accelerations(3.3 * i);
    _msg.add_effort(4.4 * i);
  }
  auto time_from_start = _msg.mutable_time_from_start();
  time_from_start->set_sec(12345);
  time_from_start->set_nsec(67890);
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::JointTrajectoryPoint> & _msg)
{
  ignition::msgs::JointTrajectoryPoint expected_msg;
  createTestMsg(expected_msg);

  for (int i = 0; i < _msg->positions_size(); ++i) {
    EXPECT_EQ(expected_msg.positions(i), _msg->positions(i));
  }

  for (int i = 0; i < _msg->velocities_size(); ++i) {
    EXPECT_EQ(expected_msg.velocities(i), _msg->velocities(i));
  }

  for (int i = 0; i < _msg->accelerations_size(); ++i) {
    EXPECT_EQ(expected_msg.accelerations(i), _msg->accelerations(i));
  }

  for (int i = 0; i < _msg->effort_size(); ++i) {
    EXPECT_EQ(expected_msg.effort(i), _msg->effort(i));
  }

  EXPECT_EQ(expected_msg.time_from_start().sec(), _msg->time_from_start().sec());
  EXPECT_EQ(expected_msg.time_from_start().nsec(), _msg->time_from_start().nsec());
}

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::JointTrajectory & _msg)
{
  const auto number_of_joints = 7;
  const auto number_of_trajectory_points = 10;

  ignition::msgs::Header header_msg;
  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.add_joint_names("joint_" + std::to_string(i));
  }

  for (auto j = 0; j < number_of_trajectory_points; ++j) {
    ignition::msgs::JointTrajectoryPoint point;
    createTestMsg(point);
    _msg.add_points();
    _msg.mutable_points(j)->CopyFrom(point);
  }
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::JointTrajectory> & _msg)
{
  ignition::msgs::JointTrajectory expected_msg;
  createTestMsg(expected_msg);

  ASSERT_TRUE(expected_msg.has_header());
  ASSERT_TRUE(_msg->has_header());

  for (int i = 0; i < _msg->joint_names_size(); ++i) {
    EXPECT_EQ(expected_msg.joint_names(i), _msg->joint_names(i));
  }

  for (int i = 0; i < _msg->points_size(); ++i) {
    compareTestMsg(std::make_shared<ignition::msgs::JointTrajectoryPoint>(_msg->points(i)));
  }
}

}  // namespace testing
}  // namespace ros_ign_bridge

#endif  // TEST_UTILS_HPP_
