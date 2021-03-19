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

#include <thread>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "../test_utils.h"

//////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ros_string_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  // std_msgs::Bool.
  ros::Publisher bool_pub = n.advertise<std_msgs::Bool>("bool", 1000);
  std_msgs::Bool bool_msg;
  ros_ign_bridge::testing::createTestMsg(bool_msg);

  // std_msgs::ColorRGBA.
  ros::Publisher color_pub = n.advertise<std_msgs::ColorRGBA>("color", 1000);
  std_msgs::ColorRGBA color_msg;
  ros_ign_bridge::testing::createTestMsg(color_msg);

  // std_msgs::Empty.
  ros::Publisher empty_pub = n.advertise<std_msgs::Empty>("empty", 1000);
  std_msgs::Empty empty_msg;

  // std_msgs::Int32.
  ros::Publisher int32_pub = n.advertise<std_msgs::Int32>("int32", 1000);
  std_msgs::Int32 int32_msg;
  ros_ign_bridge::testing::createTestMsg(int32_msg);

  // std_msgs::Float32.
  ros::Publisher float_pub = n.advertise<std_msgs::Float32>("float", 1000);
  std_msgs::Float32 float_msg;
  ros_ign_bridge::testing::createTestMsg(float_msg);

  // std_msgs::Float64.
  ros::Publisher double_pub = n.advertise<std_msgs::Float64>("double", 1000);
  std_msgs::Float64 double_msg;
  ros_ign_bridge::testing::createTestMsg(double_msg);

  // std_msgs::Header.
  ros::Publisher header_pub = n.advertise<std_msgs::Header>("header", 1000);
  std_msgs::Header header_msg;
  ros_ign_bridge::testing::createTestMsg(header_msg);

  // std_msgs::String.
  ros::Publisher string_pub = n.advertise<std_msgs::String>("string", 1000);
  std_msgs::String string_msg;
  ros_ign_bridge::testing::createTestMsg(string_msg);

  // geometry_msgs::Quaternion.
  ros::Publisher quaternion_pub =
    n.advertise<geometry_msgs::Quaternion>("quaternion", 1000);
  geometry_msgs::Quaternion quaternion_msg;
  ros_ign_bridge::testing::createTestMsg(quaternion_msg);

  // geometry_msgs::Vector3.
  ros::Publisher vector3_pub =
    n.advertise<geometry_msgs::Vector3>("vector3", 1000);
  geometry_msgs::Vector3 vector3_msg;
  ros_ign_bridge::testing::createTestMsg(vector3_msg);

  // sensor_msgs::Clock.
  ros::Publisher clock_pub =
    n.advertise<rosgraph_msgs::Clock>("clock", 1000);
  rosgraph_msgs::Clock clock_msg;
  ros_ign_bridge::testing::createTestMsg(clock_msg);

  // geometry_msgs::Point.
  ros::Publisher point_pub =
    n.advertise<geometry_msgs::Point>("point", 1000);
  geometry_msgs::Point point_msg;
  ros_ign_bridge::testing::createTestMsg(point_msg);

  // geometry_msgs::Pose.
  ros::Publisher pose_pub =
    n.advertise<geometry_msgs::Pose>("pose", 1000);
  geometry_msgs::Pose pose_msg;
  ros_ign_bridge::testing::createTestMsg(pose_msg);

  // geometry_msgs::PoseArray.
  ros::Publisher pose_array_pub =
    n.advertise<geometry_msgs::PoseArray>("pose_array", 1000);
  geometry_msgs::PoseArray pose_array_msg;
  ros_ign_bridge::testing::createTestMsg(pose_array_msg);

  // geometry_msgs::PoseStamped.
  ros::Publisher pose_stamped_pub =
    n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1000);
  geometry_msgs::PoseStamped pose_stamped_msg;
  ros_ign_bridge::testing::createTestMsg(pose_stamped_msg);

  // geometry_msgs::Transform.
  ros::Publisher transform_pub =
    n.advertise<geometry_msgs::Transform>("transform", 1000);
  geometry_msgs::Transform transform_msg;
  ros_ign_bridge::testing::createTestMsg(transform_msg);

  // geometry_msgs::TransformStamped.
  ros::Publisher transform_stamped_pub =
    n.advertise<geometry_msgs::TransformStamped>("transform_stamped", 1000);
  geometry_msgs::TransformStamped transform_stamped_msg;
  ros_ign_bridge::testing::createTestMsg(transform_stamped_msg);

  // tf2_msgs::TFMessage.
  ros::Publisher tf2_message_pub =
    n.advertise<tf2_msgs::TFMessage>("tf2_message", 1000);
  tf2_msgs::TFMessage tf2_msg;
  ros_ign_bridge::testing::createTestMsg(tf2_msg);

  // geometry_msgs::Twist.
  ros::Publisher twist_pub =
    n.advertise<geometry_msgs::Twist>("twist", 1000);
  geometry_msgs::Twist twist_msg;
  ros_ign_bridge::testing::createTestMsg(twist_msg);

  // mav_msgs::Actuators.
  ros::Publisher actuators_pub =
    n.advertise<mav_msgs::Actuators>("actuators", 1000);
  mav_msgs::Actuators actuators_msg;
  ros_ign_bridge::testing::createTestMsg(actuators_msg);

  // nav_msgs::OccupancyGrid.
  ros::Publisher map_pub =
    n.advertise<nav_msgs::OccupancyGrid>("map", 1000);
  nav_msgs::OccupancyGrid map_msg;
  ros_ign_bridge::testing::createTestMsg(map_msg);

  // nav_msgs::Odometry.
  ros::Publisher odometry_pub =
    n.advertise<nav_msgs::Odometry>("odometry", 1000);
  nav_msgs::Odometry odometry_msg;
  ros_ign_bridge::testing::createTestMsg(odometry_msg);

  // sensor_msgs::Image.
  ros::Publisher image_pub =
    n.advertise<sensor_msgs::Image>("image", 1000);
  sensor_msgs::Image image_msg;
  ros_ign_bridge::testing::createTestMsg(image_msg);

  // sensor_msgs::CameraInfo.
  ros::Publisher camera_info_pub =
    n.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
  sensor_msgs::CameraInfo camera_info_msg;
  ros_ign_bridge::testing::createTestMsg(camera_info_msg);

  // sensor_msgs::FluidPressure.
  ros::Publisher fluid_pressure_pub =
    n.advertise<sensor_msgs::FluidPressure>("fluid_pressure", 1000);
  sensor_msgs::FluidPressure fluid_pressure_msg;
  ros_ign_bridge::testing::createTestMsg(fluid_pressure_msg);

  // sensor_msgs::Imu.
  ros::Publisher imu_pub =
    n.advertise<sensor_msgs::Imu>("imu", 1000);
  sensor_msgs::Imu imu_msg;
  ros_ign_bridge::testing::createTestMsg(imu_msg);

  // sensor_msgs::JointState.
  ros::Publisher joint_states_pub =
    n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  sensor_msgs::JointState joint_states_msg;
  ros_ign_bridge::testing::createTestMsg(joint_states_msg);

  // sensor_msgs::LaserScan.
  ros::Publisher laserscan_pub =
    n.advertise<sensor_msgs::LaserScan>("laserscan", 1000);
  sensor_msgs::LaserScan laserscan_msg;
  ros_ign_bridge::testing::createTestMsg(laserscan_msg);

  // sensor_msgs::MagneticField.
  ros::Publisher magnetic_pub =
    n.advertise<sensor_msgs::MagneticField>("magnetic", 1000);
  sensor_msgs::MagneticField magnetic_msg;
  ros_ign_bridge::testing::createTestMsg(magnetic_msg);

  // sensor_msgs::PointCloud2.
  ros::Publisher pointcloud2_pub =
    n.advertise<sensor_msgs::PointCloud2>("pointcloud2", 1000);
  sensor_msgs::PointCloud2 pointcloud2_msg;
  ros_ign_bridge::testing::createTestMsg(pointcloud2_msg);

  // sensor_msgs::BatteryState.
  ros::Publisher battery_state_pub =
    n.advertise<sensor_msgs::BatteryState>("battery_state", 1000);
  sensor_msgs::BatteryState battery_state_msg;
  ros_ign_bridge::testing::createTestMsg(battery_state_msg);

  // visualization_msgs::Marker.
  ros::Publisher marker_pub =
    n.advertise<visualization_msgs::Marker>("marker", 1000);
  visualization_msgs::Marker marker_msg;
  ros_ign_bridge::testing::createTestMsg(marker_msg);

  // visualization_msgs::MarkerArray.
  ros::Publisher marker_array_pub =
    n.advertise<visualization_msgs::MarkerArray>("marker_array", 1000);
  visualization_msgs::MarkerArray marker_array_msg;
  ros_ign_bridge::testing::createTestMsg(marker_array_msg);

  while (ros::ok())
  {
    // Publish all messages.
    bool_pub.publish(bool_msg);
    color_pub.publish(color_msg);
    empty_pub.publish(empty_msg);
    int32_pub.publish(int32_msg);
    float_pub.publish(float_msg);
    double_pub.publish(double_msg);
    header_pub.publish(header_msg);
    string_pub.publish(string_msg);
    quaternion_pub.publish(quaternion_msg);
    vector3_pub.publish(vector3_msg);
    clock_pub.publish(clock_msg);
    point_pub.publish(point_msg);
    pose_pub.publish(pose_msg);
    pose_array_pub.publish(pose_array_msg);
    pose_stamped_pub.publish(pose_stamped_msg);
    transform_pub.publish(transform_msg);
    transform_stamped_pub.publish(transform_stamped_msg);
    tf2_message_pub.publish(tf2_msg);
    twist_pub.publish(twist_msg);
    actuators_pub.publish(actuators_msg);
    map_pub.publish(map_msg);
    odometry_pub.publish(odometry_msg);
    image_pub.publish(image_msg);
    camera_info_pub.publish(camera_info_msg);
    fluid_pressure_pub.publish(fluid_pressure_msg);
    imu_pub.publish(imu_msg);
    laserscan_pub.publish(laserscan_msg);
    magnetic_pub.publish(magnetic_msg);
    joint_states_pub.publish(joint_states_msg);
    pointcloud2_pub.publish(pointcloud2_msg);
    battery_state_pub.publish(battery_state_msg);
    marker_pub.publish(marker_msg);
    marker_array_pub.publish(marker_array_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
