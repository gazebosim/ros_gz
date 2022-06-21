// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "utils/test_utils.hpp"
#include "utils/gz_test_msg.hpp"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM) {
    g_terminatePub = true;
  }
}

//////////////////////////////////////////////////
int main(int /*argc*/, char **/*argv*/)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  ignition::transport::Node node;

  // ignition::msgs::Color.
  auto color_pub = node.Advertise<ignition::msgs::Color>("color");
  ignition::msgs::Color color_msg;
  ros_gz_bridge::testing::createTestMsg(color_msg);

  // ignition::msgs::Light.
  auto light_pub = node.Advertise<ignition::msgs::Light>("light");
  ignition::msgs::Light light_msg;
  ros_gz_bridge::testing::createTestMsg(light_msg);

  // ignition::msgs::Boolean.
  auto bool_pub = node.Advertise<ignition::msgs::Boolean>("bool");
  ignition::msgs::Boolean bool_msg;
  ros_gz_bridge::testing::createTestMsg(bool_msg);

  // ignition::msgs::Empty.
  auto empty_pub = node.Advertise<ignition::msgs::Empty>("empty");
  ignition::msgs::Empty empty_msg;

  // ignition::msgs::Float.
  auto float_pub = node.Advertise<ignition::msgs::Float>("float");
  ignition::msgs::Float float_msg;
  ros_gz_bridge::testing::createTestMsg(float_msg);

  // ignition::msgs::Double.
  auto double_pub = node.Advertise<ignition::msgs::Double>("double");
  ignition::msgs::Double double_msg;
  ros_gz_bridge::testing::createTestMsg(double_msg);

  // ignition::msgs::Uint32.
  auto uint32_pub = node.Advertise<ignition::msgs::UInt32>("uint32");
  ignition::msgs::UInt32 uint32_msg;
  ros_gz_bridge::testing::createTestMsg(uint32_msg);

  // ignition::msgs::Header.
  auto header_pub = node.Advertise<ignition::msgs::Header>("header");
  ignition::msgs::Header header_msg;
  ros_gz_bridge::testing::createTestMsg(header_msg);

  // ignition::msgs::StringMsg.
  auto string_pub = node.Advertise<ignition::msgs::StringMsg>("string");
  ignition::msgs::StringMsg string_msg;
  ros_gz_bridge::testing::createTestMsg(string_msg);

  // ignition::msgs::Quaternion.
  auto quaternion_pub =
    node.Advertise<ignition::msgs::Quaternion>("quaternion");
  ignition::msgs::Quaternion quaternion_msg;
  ros_gz_bridge::testing::createTestMsg(quaternion_msg);

  // ignition::msgs::Vector3d.
  auto vector3_pub = node.Advertise<ignition::msgs::Vector3d>("vector3");
  ignition::msgs::Vector3d vector3_msg;
  ros_gz_bridge::testing::createTestMsg(vector3_msg);

  // ignition::msgs::Clock.
  auto clock_pub = node.Advertise<ignition::msgs::Clock>("clock");
  ignition::msgs::Clock clock_msg;
  ros_gz_bridge::testing::createTestMsg(clock_msg);

  // ignition::msgs::Point.
  auto point_pub = node.Advertise<ignition::msgs::Vector3d>("point");
  ignition::msgs::Vector3d point_msg;
  ros_gz_bridge::testing::createTestMsg(point_msg);

  // ignition::msgs::Pose.
  auto pose_pub = node.Advertise<ignition::msgs::Pose>("pose");
  ignition::msgs::Pose pose_msg;
  ros_gz_bridge::testing::createTestMsg(pose_msg);

  // ignition::msgs::PoseWithCovariance.
  auto pose_cov_pub = node.Advertise<ignition::msgs::PoseWithCovariance>("pose_with_covariance");
  ignition::msgs::PoseWithCovariance pose_cov_msg;
  ros_gz_bridge::testing::createTestMsg(pose_cov_msg);

  // ignition::msgs::PoseStamped.
  auto pose_stamped_pub = node.Advertise<ignition::msgs::Pose>("pose_stamped");
  ignition::msgs::Pose pose_stamped_msg;
  ros_gz_bridge::testing::createTestMsg(pose_stamped_msg);

  // ignition::msgs::Transform.
  auto transform_pub =
    node.Advertise<ignition::msgs::Pose>("transform");
  ignition::msgs::Pose transform_msg;
  ros_gz_bridge::testing::createTestMsg(transform_msg);

  // ignition::msgs::TransformStamped.
  auto transform_stamped_pub =
    node.Advertise<ignition::msgs::Pose>("transform_stamped");
  ignition::msgs::Pose transform_stamped_msg;
  ros_gz_bridge::testing::createTestMsg(transform_stamped_msg);

  // ignition::msgs::Pose_V.
  auto tf2_message_pub =
    node.Advertise<ignition::msgs::Pose_V>("tf2_message");
  ignition::msgs::Pose_V tf2_msg;
  ros_gz_bridge::testing::createTestMsg(tf2_msg);

  // ignition::msgs::Image.
  auto image_pub = node.Advertise<ignition::msgs::Image>("image");
  ignition::msgs::Image image_msg;
  ros_gz_bridge::testing::createTestMsg(image_msg);

  // ignition::msgs::CameraInfo.
  auto camera_info_pub = node.Advertise<ignition::msgs::CameraInfo>("camera_info");
  ignition::msgs::CameraInfo camera_info_msg;
  ros_gz_bridge::testing::createTestMsg(camera_info_msg);

  // ignition::msgs::FluidPressure.
  auto fluid_pressure_pub = node.Advertise<ignition::msgs::FluidPressure>("fluid_pressure");
  ignition::msgs::FluidPressure fluid_pressure_msg;
  ros_gz_bridge::testing::createTestMsg(fluid_pressure_msg);

  // ignition::msgs::IMU.
  auto imu_pub = node.Advertise<ignition::msgs::IMU>("imu");
  ignition::msgs::IMU imu_msg;
  ros_gz_bridge::testing::createTestMsg(imu_msg);

  // ignition::msgs::LaserScan.
  auto laserscan_pub = node.Advertise<ignition::msgs::LaserScan>("laserscan");
  ignition::msgs::LaserScan laserscan_msg;
  ros_gz_bridge::testing::createTestMsg(laserscan_msg);

  // ignition::msgs::Magnetometer.
  auto magnetic_pub = node.Advertise<ignition::msgs::Magnetometer>("magnetic");
  ignition::msgs::Magnetometer magnetometer_msg;
  ros_gz_bridge::testing::createTestMsg(magnetometer_msg);

  // ignition::msgs::Actuators.
  auto actuators_pub = node.Advertise<ignition::msgs::Actuators>("actuators");
  ignition::msgs::Actuators actuators_msg;
  ros_gz_bridge::testing::createTestMsg(actuators_msg);

  // ignition::msgs::Odometry.
  auto odometry_pub = node.Advertise<ignition::msgs::Odometry>("odometry");
  ignition::msgs::Odometry odometry_msg;
  ros_gz_bridge::testing::createTestMsg(odometry_msg);

  // ignition::msgs::OdometryWithCovariance.
  auto odometry_cov_pub = node.Advertise<ignition::msgs::OdometryWithCovariance>(
    "odometry_with_covariance");
  ignition::msgs::OdometryWithCovariance odometry_cov_msg;
  ros_gz_bridge::testing::createTestMsg(odometry_cov_msg);

  // ignition::msgs::Model.
  auto joint_states_pub = node.Advertise<ignition::msgs::Model>("joint_states");
  ignition::msgs::Model joint_states_msg;
  ros_gz_bridge::testing::createTestMsg(joint_states_msg);

  // ignition::msgs::Twist.
  auto twist_pub = node.Advertise<ignition::msgs::Twist>("twist");
  ignition::msgs::Twist twist_msg;
  ros_gz_bridge::testing::createTestMsg(twist_msg);

  // ignition::msgs::TwistWithCovariance.
  auto twist_cov_pub = node.Advertise<ignition::msgs::TwistWithCovariance>(
    "twist_with_covariance");
  ignition::msgs::TwistWithCovariance twist_cov_msg;
  ros_gz_bridge::testing::createTestMsg(twist_cov_msg);

  // ignition::msgs::Wrench.
  auto wrench_pub = node.Advertise<ignition::msgs::Wrench>("wrench");
  ignition::msgs::Wrench wrench_msg;
  ros_gz_bridge::testing::createTestMsg(wrench_msg);

  // ignition::msgs::JointWrench.
  auto joint_wrench_pub = node.Advertise<ignition::msgs::JointWrench>("joint_wrench");
  ignition::msgs::JointWrench joint_wrench_msg;
  ros_gz_bridge::testing::createTestMsg(joint_wrench_msg);

  // ignition::msgs::Entity.
  auto entity_pub = node.Advertise<ignition::msgs::Entity>("entity");
  ignition::msgs::Entity entity_msg;
  ros_gz_bridge::testing::createTestMsg(entity_msg);

  // ignition::msgs::Contact.
  auto contact_pub = node.Advertise<ignition::msgs::Contact>("contact");
  ignition::msgs::Contact contact_msg;
  ros_gz_bridge::testing::createTestMsg(contact_msg);

  // ignition::msgs::Contacts.
  auto contacts_pub = node.Advertise<ignition::msgs::Contacts>("contacts");
  ignition::msgs::Contacts contacts_msg;
  ros_gz_bridge::testing::createTestMsg(contacts_msg);

  // ignition::msgs::PointCloudPacked.
  auto pointcloudpacked_pub = node.Advertise<ignition::msgs::PointCloudPacked>(
    "pointcloud2");
  ignition::msgs::PointCloudPacked pointcloudpacked_msg;
  ros_gz_bridge::testing::createTestMsg(pointcloudpacked_msg);

  // ignition::msgs::BatteryState.
  auto battery_state_pub = node.Advertise<ignition::msgs::BatteryState>("battery_state");
  ignition::msgs::BatteryState battery_state_msg;
  ros_gz_bridge::testing::createTestMsg(battery_state_msg);

  // ignition::msgs::JointTrajectory.
  auto joint_trajectory_pub = node.Advertise<ignition::msgs::JointTrajectory>("joint_trajectory");
  ignition::msgs::JointTrajectory joint_trajectory_msg;
  ros_gz_bridge::testing::createTestMsg(joint_trajectory_msg);

  // ignition::msgs::GUICamera.
  auto gui_camera_pub = node.Advertise<ignition::msgs::GUICamera>("gui_camera");
  ignition::msgs::GUICamera gui_camera_msg;
  ros_gz_bridge::testing::createTestMsg(gui_camera_msg);

  // ignition::msgs::StringMsg_V.
  auto stringmsg_v_pub = node.Advertise<ignition::msgs::StringMsg_V>("stringmsg_v");
  ignition::msgs::StringMsg_V stringmsg_v_msg;
  ros_gz_bridge::testing::createTestMsg(stringmsg_v_msg);

  // ignition::msgs::Time.
  auto time_pub = node.Advertise<ignition::msgs::Time>("time");
  ignition::msgs::Time time_msg;
  ros_gz_bridge::testing::createTestMsg(time_msg);

  // ignition::msgs::TrackVisual.
  auto track_visual_pub = node.Advertise<ignition::msgs::TrackVisual>("track_visual");
  ignition::msgs::TrackVisual track_visual_msg;
  ros_gz_bridge::testing::createTestMsg(track_visual_msg);

  // ignition::msgs::VideoRecord.
  auto video_record_pub = node.Advertise<ignition::msgs::VideoRecord>("video_record");
  ignition::msgs::VideoRecord video_record_msg;
  ros_gz_bridge::testing::createTestMsg(video_record_msg);

  // Publish messages at 100Hz.
  while (!g_terminatePub) {
    color_pub.Publish(color_msg);
    light_pub.Publish(light_msg);
    bool_pub.Publish(bool_msg);
    empty_pub.Publish(empty_msg);
    float_pub.Publish(float_msg);
    double_pub.Publish(double_msg);
    uint32_pub.Publish(uint32_msg);
    header_pub.Publish(header_msg);
    string_pub.Publish(string_msg);
    quaternion_pub.Publish(quaternion_msg);
    vector3_pub.Publish(vector3_msg);
    clock_pub.Publish(clock_msg);
    point_pub.Publish(point_msg);
    pose_pub.Publish(pose_msg);
    pose_cov_pub.Publish(pose_cov_msg);
    pose_stamped_pub.Publish(pose_stamped_msg);
    transform_pub.Publish(transform_msg);
    transform_stamped_pub.Publish(transform_stamped_msg);
    tf2_message_pub.Publish(tf2_msg);
    wrench_pub.Publish(wrench_msg);
    joint_wrench_pub.Publish(joint_wrench_msg);
    entity_pub.Publish(entity_msg);
    contact_pub.Publish(contact_msg);
    contacts_pub.Publish(contacts_msg);
    image_pub.Publish(image_msg);
    camera_info_pub.Publish(camera_info_msg);
    fluid_pressure_pub.Publish(fluid_pressure_msg);
    imu_pub.Publish(imu_msg);
    laserscan_pub.Publish(laserscan_msg);
    magnetic_pub.Publish(magnetometer_msg);
    actuators_pub.Publish(actuators_msg);
    odometry_pub.Publish(odometry_msg);
    odometry_cov_pub.Publish(odometry_cov_msg);
    joint_states_pub.Publish(joint_states_msg);
    twist_pub.Publish(twist_msg);
    twist_cov_pub.Publish(twist_cov_msg);
    pointcloudpacked_pub.Publish(pointcloudpacked_msg);
    battery_state_pub.Publish(battery_state_msg);
    joint_trajectory_pub.Publish(joint_trajectory_msg);
    gui_camera_pub.Publish(gui_camera_msg);
    stringmsg_v_pub.Publish(stringmsg_v_msg);
    time_pub.Publish(time_msg);
    track_visual_pub.Publish(track_visual_msg);
    video_record_pub.Publish(video_record_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
