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

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "../test_utils.h"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

//////////////////////////////////////////////////
int main(int /*argc*/, char **/*argv*/)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  ignition::transport::Node node;

  // ignition::msgs::Boolean.
  auto bool_pub = node.Advertise<ignition::msgs::Boolean>("bool");
  ignition::msgs::Boolean bool_msg;
  ros_ign_bridge::testing::createTestMsg(bool_msg);

  // ignition::msgs::Color.
  auto color_pub = node.Advertise<ignition::msgs::Color>("color");
  ignition::msgs::Color color_msg;
  ros_ign_bridge::testing::createTestMsg(color_msg);

  // ignition::msgs::Empty.
  auto empty_pub = node.Advertise<ignition::msgs::Empty>("empty");
  ignition::msgs::Empty empty_msg;

  // ignition::msgs::Int32.
  auto int32_pub = node.Advertise<ignition::msgs::Int32>("int32");
  ignition::msgs::Int32 int32_msg;
  ros_ign_bridge::testing::createTestMsg(int32_msg);

  // ignition::msgs::Float.
  auto float_pub = node.Advertise<ignition::msgs::Float>("float");
  ignition::msgs::Float float_msg;
  ros_ign_bridge::testing::createTestMsg(float_msg);

  // ignition::msgs::Double.
  auto double_pub = node.Advertise<ignition::msgs::Double>("double");
  ignition::msgs::Double double_msg;
  ros_ign_bridge::testing::createTestMsg(double_msg);

  // ignition::msgs::Header.
  auto header_pub = node.Advertise<ignition::msgs::Header>("header");
  ignition::msgs::Header header_msg;
  ros_ign_bridge::testing::createTestMsg(header_msg);

  // ignition::msgs::StringMsg.
  auto string_pub = node.Advertise<ignition::msgs::StringMsg>("string");
  ignition::msgs::StringMsg string_msg;
  ros_ign_bridge::testing::createTestMsg(string_msg);

  // ignition::msgs::Quaternion.
  auto quaternion_pub =
    node.Advertise<ignition::msgs::Quaternion>("quaternion");
  ignition::msgs::Quaternion quaternion_msg;
  ros_ign_bridge::testing::createTestMsg(quaternion_msg);

  // ignition::msgs::Vector3d.
  auto vector3_pub = node.Advertise<ignition::msgs::Vector3d>("vector3");
  ignition::msgs::Vector3d vector3_msg;
  ros_ign_bridge::testing::createTestMsg(vector3_msg);

  // ignition::msgs::Clock.
  auto clock_pub = node.Advertise<ignition::msgs::Clock>("clock");
  ignition::msgs::Clock clock_msg;
  ros_ign_bridge::testing::createTestMsg(clock_msg);

  // ignition::msgs::Point.
  auto point_pub = node.Advertise<ignition::msgs::Vector3d>("point");
  ignition::msgs::Vector3d point_msg;
  ros_ign_bridge::testing::createTestMsg(point_msg);

  // ignition::msgs::Pose.
  auto pose_pub = node.Advertise<ignition::msgs::Pose>("pose");
  ignition::msgs::Pose pose_msg;
  ros_ign_bridge::testing::createTestMsg(pose_msg);

  // ignition::msgs::PoseStamped.
  auto pose_stamped_pub = node.Advertise<ignition::msgs::Pose>("pose_stamped");
  ignition::msgs::Pose pose_stamped_msg;
  ros_ign_bridge::testing::createTestMsg(pose_stamped_msg);

  // ignition::msgs::Pose_V.
  auto pose_v_pub = node.Advertise<ignition::msgs::Pose_V>("pose_array");
  ignition::msgs::Pose_V pose_v_msg;
  ros_ign_bridge::testing::createTestMsg(pose_v_msg);

  // ignition::msgs::Transform.
  auto transform_pub =
      node.Advertise<ignition::msgs::Pose>("transform");
  ignition::msgs::Pose transform_msg;
  ros_ign_bridge::testing::createTestMsg(transform_msg);

  // ignition::msgs::TransformStamped.
  auto transform_stamped_pub =
      node.Advertise<ignition::msgs::Pose>("transform_stamped");
  ignition::msgs::Pose transform_stamped_msg;
  ros_ign_bridge::testing::createTestMsg(transform_stamped_msg);

  // ignition::msgs::Pose_V.
  auto tf2_message_pub =
      node.Advertise<ignition::msgs::Pose_V>("tf2_message");
  ignition::msgs::Pose_V tf2_msg;
  ros_ign_bridge::testing::createTestMsg(tf2_msg);

  // ignition::msgs::Image.
  auto image_pub = node.Advertise<ignition::msgs::Image>("image");
  ignition::msgs::Image image_msg;
  ros_ign_bridge::testing::createTestMsg(image_msg);

  // ignition::msgs::CameraInfo.
  auto camera_info_pub = node.Advertise<ignition::msgs::CameraInfo>("camera_info");
  ignition::msgs::CameraInfo camera_info_msg;
  ros_ign_bridge::testing::createTestMsg(camera_info_msg);

  // ignition::msgs::FluidPressure.
  auto fluid_pressure_pub = node.Advertise<ignition::msgs::FluidPressure>("fluid_pressure");
  ignition::msgs::FluidPressure fluid_pressure_msg;
  ros_ign_bridge::testing::createTestMsg(fluid_pressure_msg);

  // ignition::msgs::IMU.
  auto imu_pub = node.Advertise<ignition::msgs::IMU>("imu");
  ignition::msgs::IMU imu_msg;
  ros_ign_bridge::testing::createTestMsg(imu_msg);

  // ignition::msgs::LaserScan.
  auto laserscan_pub = node.Advertise<ignition::msgs::LaserScan>("laserscan");
  ignition::msgs::LaserScan laserscan_msg;
  ros_ign_bridge::testing::createTestMsg(laserscan_msg);

  // ignition::msgs::Magnetometer.
  auto magnetic_pub = node.Advertise<ignition::msgs::Magnetometer>("magnetic");
  ignition::msgs::Magnetometer magnetometer_msg;
  ros_ign_bridge::testing::createTestMsg(magnetometer_msg);

  // ignition::msgs::Actuators.
  auto actuators_pub = node.Advertise<ignition::msgs::Actuators>("actuators");
  ignition::msgs::Actuators actuators_msg;
  ros_ign_bridge::testing::createTestMsg(actuators_msg);

  // ignition::msgs::OccupancyGrid
  auto map_pub = node.Advertise<ignition::msgs::OccupancyGrid>("map");
  ignition::msgs::OccupancyGrid map_msg;
  ros_ign_bridge::testing::createTestMsg(map_msg);

  // ignition::msgs::Odometry.
  auto odometry_pub = node.Advertise<ignition::msgs::Odometry>("odometry");
  ignition::msgs::Odometry odometry_msg;
  ros_ign_bridge::testing::createTestMsg(odometry_msg);

  // ignition::msgs::Model.
  auto joint_states_pub = node.Advertise<ignition::msgs::Model>("joint_states");
  ignition::msgs::Model joint_states_msg;
  ros_ign_bridge::testing::createTestMsg(joint_states_msg);

  // ignition::msgs::Twist.
  auto twist_pub = node.Advertise<ignition::msgs::Twist>("twist");
  ignition::msgs::Twist twist_msg;
  ros_ign_bridge::testing::createTestMsg(twist_msg);

  // ignition::msgs::PointCloudPacked.
  auto pointcloudpacked_pub = node.Advertise<ignition::msgs::PointCloudPacked>(
      "pointcloud2");
  ignition::msgs::PointCloudPacked pointcloudpacked_msg;
  ros_ign_bridge::testing::createTestMsg(pointcloudpacked_msg);

  // ignition::msgs::BatteryState.
  auto battery_state_pub = node.Advertise<ignition::msgs::BatteryState>("battery_state");
  ignition::msgs::BatteryState battery_state_msg;
  ros_ign_bridge::testing::createTestMsg(battery_state_msg);

  auto marker_pub = node.Advertise<ignition::msgs::Marker>("marker");
  ignition::msgs::Marker marker_msg;
  ros_ign_bridge::testing::createTestMsg(marker_msg);

  auto marker_array_pub = node.Advertise<ignition::msgs::Marker_V>("marker_array");
  ignition::msgs::Marker_V marker_array_msg;
  ros_ign_bridge::testing::createTestMsg(marker_array_msg);

  // Publish messages at 1Hz.
  while (!g_terminatePub)
  {
    bool_pub.Publish(bool_msg);
    color_pub.Publish(color_msg);
    empty_pub.Publish(empty_msg);
    int32_pub.Publish(int32_msg);
    float_pub.Publish(float_msg);
    double_pub.Publish(double_msg);
    header_pub.Publish(header_msg);
    string_pub.Publish(string_msg);
    quaternion_pub.Publish(quaternion_msg);
    vector3_pub.Publish(vector3_msg);
    clock_pub.Publish(clock_msg);
    point_pub.Publish(point_msg);
    pose_pub.Publish(pose_msg);
    pose_v_pub.Publish(pose_v_msg);
    pose_stamped_pub.Publish(pose_stamped_msg);
    transform_pub.Publish(transform_msg);
    transform_stamped_pub.Publish(transform_stamped_msg);
    tf2_message_pub.Publish(tf2_msg);
    image_pub.Publish(image_msg);
    camera_info_pub.Publish(camera_info_msg);
    fluid_pressure_pub.Publish(fluid_pressure_msg);
    imu_pub.Publish(imu_msg);
    laserscan_pub.Publish(laserscan_msg);
    magnetic_pub.Publish(magnetometer_msg);
    actuators_pub.Publish(actuators_msg);
    map_pub.Publish(map_msg);
    odometry_pub.Publish(odometry_msg);
    joint_states_pub.Publish(joint_states_msg);
    twist_pub.Publish(twist_msg);
    pointcloudpacked_pub.Publish(pointcloudpacked_msg);
    battery_state_pub.Publish(battery_state_msg);
    marker_pub.Publish(marker_msg);
    marker_array_pub.Publish(marker_array_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
