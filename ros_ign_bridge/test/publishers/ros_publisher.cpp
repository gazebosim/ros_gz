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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// #include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include "../test_utils.hpp"

//////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_string_publisher");

  rclcpp::Rate loop_rate(1);
  // std_msgs::msg::Bool.
  auto bool_pub = node->create_publisher<std_msgs::msg::Bool>("bool", 1000);
  std_msgs::msg::Bool bool_msg;
  ros_ign_bridge::testing::createTestMsg(bool_msg);

  // std_msgs::msg::Empty.
  auto empty_pub = node->create_publisher<std_msgs::msg::Empty>("empty", 1000);
  std_msgs::msg::Empty empty_msg;

  // std_msgs::msg::Float32.
  auto float_pub = node->create_publisher<std_msgs::msg::Float32>("float", 1000);
  std_msgs::msg::Float32 float_msg;
  ros_ign_bridge::testing::createTestMsg(float_msg);

  // std_msgs::msg::Header.
  auto header_pub = node->create_publisher<std_msgs::msg::Header>("header", 1000);
  std_msgs::msg::Header header_msg;
  ros_ign_bridge::testing::createTestMsg(header_msg);

  // std_msgs::msg::String.
  auto string_pub = node->create_publisher<std_msgs::msg::String>("string", 1000);
  std_msgs::msg::String string_msg;
  ros_ign_bridge::testing::createTestMsg(string_msg);

  // geometry_msgs::msg::Quaternion.
  auto quaternion_pub =
    node->create_publisher<geometry_msgs::msg::Quaternion>("quaternion", 1000);
  geometry_msgs::msg::Quaternion quaternion_msg;
  ros_ign_bridge::testing::createTestMsg(quaternion_msg);

  // geometry_msgs::msg::Vector3.
  auto vector3_pub =
    node->create_publisher<geometry_msgs::msg::Vector3>("vector3", 1000);
  geometry_msgs::msg::Vector3 vector3_msg;
  ros_ign_bridge::testing::createTestMsg(vector3_msg);

  // sensor_msgs::msg::Clock.
  auto clock_pub =
    node->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1000);
  rosgraph_msgs::msg::Clock clock_msg;
  ros_ign_bridge::testing::createTestMsg(clock_msg);

  // geometry_msgs::msg::Point.
  auto point_pub =
    node->create_publisher<geometry_msgs::msg::Point>("point", 1000);
  geometry_msgs::msg::Point point_msg;
  ros_ign_bridge::testing::createTestMsg(point_msg);

  // geometry_msgs::msg::Pose.
  auto pose_pub =
    node->create_publisher<geometry_msgs::msg::Pose>("pose", 1000);
  geometry_msgs::msg::Pose pose_msg;
  ros_ign_bridge::testing::createTestMsg(pose_msg);

  // geometry_msgs::msg::PoseStamped.
  auto pose_stamped_pub =
    node->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 1000);
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  ros_ign_bridge::testing::createTestMsg(pose_stamped_msg);

  // geometry_msgs::msg::Transform.
  auto transform_pub =
    node->create_publisher<geometry_msgs::msg::Transform>("transform", 1000);
  geometry_msgs::msg::Transform transform_msg;
  ros_ign_bridge::testing::createTestMsg(transform_msg);

  // geometry_msgs::msg::TransformStamped.
  auto transform_stamped_pub =
    node->create_publisher<geometry_msgs::msg::TransformStamped>("transform_stamped", 1000);
  geometry_msgs::msg::TransformStamped transform_stamped_msg;
  ros_ign_bridge::testing::createTestMsg(transform_stamped_msg);

  // geometry_msgs::msg::Twist.
  auto twist_pub =
    node->create_publisher<geometry_msgs::msg::Twist>("twist", 1000);
  geometry_msgs::msg::Twist twist_msg;
  ros_ign_bridge::testing::createTestMsg(twist_msg);

  // // mav_msgs::msg::Actuators.
  // auto actuators_pub =
  //   node->create_publisher<mav_msgs::msg::Actuators>("actuators", 1000);
  // mav_msgs::msg::Actuators actuators_msg;
  // ros_ign_bridge::testing::createTestMsg(actuators_msg);

  // nav_msgs::msg::Odometry.
  auto odometry_pub =
    node->create_publisher<nav_msgs::msg::Odometry>("odometry", 1000);
  nav_msgs::msg::Odometry odometry_msg;
  ros_ign_bridge::testing::createTestMsg(odometry_msg);

  // sensor_msgs::msg::Image.
  auto image_pub =
    node->create_publisher<sensor_msgs::msg::Image>("image", 1000);
  sensor_msgs::msg::Image image_msg;
  ros_ign_bridge::testing::createTestMsg(image_msg);

  // sensor_msgs::msg::CameraInfo.
  auto camera_info_pub =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1000);
  sensor_msgs::msg::CameraInfo camera_info_msg;
  ros_ign_bridge::testing::createTestMsg(camera_info_msg);

  // sensor_msgs::msg::FluidPressure.
  auto fluid_pressure_pub =
    node->create_publisher<sensor_msgs::msg::FluidPressure>("fluid_pressure", 1000);
  sensor_msgs::msg::FluidPressure fluid_pressure_msg;
  ros_ign_bridge::testing::createTestMsg(fluid_pressure_msg);

  // sensor_msgs::msg::Imu.
  auto imu_pub =
    node->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
  sensor_msgs::msg::Imu imu_msg;
  ros_ign_bridge::testing::createTestMsg(imu_msg);

  // sensor_msgs::msg::JointState.
  auto joint_states_pub =
    node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);
  sensor_msgs::msg::JointState joint_states_msg;
  ros_ign_bridge::testing::createTestMsg(joint_states_msg);

  // sensor_msgs::msg::LaserScan.
  auto laserscan_pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", 1000);
  sensor_msgs::msg::LaserScan laserscan_msg;
  ros_ign_bridge::testing::createTestMsg(laserscan_msg);

  // sensor_msgs::msg::MagneticField.
  auto magnetic_pub =
    node->create_publisher<sensor_msgs::msg::MagneticField>("magnetic", 1000);
  sensor_msgs::msg::MagneticField magnetic_msg;
  ros_ign_bridge::testing::createTestMsg(magnetic_msg);

  // sensor_msgs::msg::PointCloud2.
  auto pointcloud2_pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud2", 1000);
  sensor_msgs::msg::PointCloud2 pointcloud2_msg;
  ros_ign_bridge::testing::createTestMsg(pointcloud2_msg);

  // sensor_msgs::msg::BatteryState.
  auto battery_state_pub =
    node->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1000);
  sensor_msgs::msg::BatteryState battery_state_msg;
  ros_ign_bridge::testing::createTestMsg(battery_state_msg);

  while (rclcpp::ok()) {
    // Publish all messages.
    bool_pub->publish(bool_msg);
    empty_pub->publish(empty_msg);
    float_pub->publish(float_msg);
    header_pub->publish(header_msg);
    string_pub->publish(string_msg);
    quaternion_pub->publish(quaternion_msg);
    vector3_pub->publish(vector3_msg);
    clock_pub->publish(clock_msg);
    point_pub->publish(point_msg);
    pose_pub->publish(pose_msg);
    pose_stamped_pub->publish(pose_stamped_msg);
    transform_pub->publish(transform_msg);
    transform_stamped_pub->publish(transform_stamped_msg);
    twist_pub->publish(twist_msg);
    // actuators_pub->publish(actuators_msg);
    odometry_pub->publish(odometry_msg);
    image_pub->publish(image_msg);
    camera_info_pub->publish(camera_info_msg);
    fluid_pressure_pub->publish(fluid_pressure_msg);
    imu_pub->publish(imu_msg);
    laserscan_pub->publish(laserscan_msg);
    magnetic_pub->publish(magnetic_msg);
    joint_states_pub->publish(joint_states_msg);
    pointcloud2_pub->publish(pointcloud2_msg);
    battery_state_pub->publish(battery_state_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
