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
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include "../test_utils.h"

//////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ros1_string_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  // std_msgs::Header.
  ros::Publisher header_pub = n.advertise<std_msgs::Header>("header", 1000);
  std_msgs::Header header_msg;
  ros1_ign_bridge::testing::createTestMsg(header_msg);

  // std_msgs::String.
  ros::Publisher string_pub = n.advertise<std_msgs::String>("string", 1000);
  std_msgs::String string_msg;
  ros1_ign_bridge::testing::createTestMsg(string_msg);

  // geometry_msgs::Quaternion.
  ros::Publisher quaternion_pub =
    n.advertise<geometry_msgs::Quaternion>("quaternion", 1000);
  geometry_msgs::Quaternion quaternion_msg;
  ros1_ign_bridge::testing::createTestMsg(quaternion_msg);

  // geometry_msgs::Vector3.
  ros::Publisher vector3_pub =
    n.advertise<geometry_msgs::Vector3>("vector3", 1000);
  geometry_msgs::Vector3 vector3_msg;
  ros1_ign_bridge::testing::createTestMsg(vector3_msg);

  // geometry_msgs::Point.
  ros::Publisher point_pub =
    n.advertise<geometry_msgs::Point>("point", 1000);
  geometry_msgs::Point point_msg;
  ros1_ign_bridge::testing::createTestMsg(point_msg);

  // geometry_msgs::Pose.
  ros::Publisher pose_pub =
    n.advertise<geometry_msgs::Pose>("pose", 1000);
  geometry_msgs::Pose pose_msg;
  ros1_ign_bridge::testing::createTestMsg(pose_msg);

  // geometry_msgs::PoseStamped.
  ros::Publisher pose_stamped_pub =
    n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1000);
  geometry_msgs::PoseStamped pose_stamped_msg;
  ros1_ign_bridge::testing::createTestMsg(pose_stamped_msg);

  // geometry_msgs::Transform.
  ros::Publisher transform_pub =
    n.advertise<geometry_msgs::Transform>("transform", 1000);
  geometry_msgs::Transform transform_msg;
  ros1_ign_bridge::testing::createTestMsg(transform_msg);

  // geometry_msgs::TransformStamped.
  ros::Publisher transform_stamped_pub =
    n.advertise<geometry_msgs::TransformStamped>("transform_stamped", 1000);
  geometry_msgs::TransformStamped transform_stamped_msg;
  ros1_ign_bridge::testing::createTestMsg(transform_stamped_msg);

  // sensor_msgs::Image.
  ros::Publisher image_pub =
    n.advertise<sensor_msgs::Image>("image", 1000);
  sensor_msgs::Image image_msg;
  ros1_ign_bridge::testing::createTestMsg(image_msg);

  // sensor_msgs::Imu.
  ros::Publisher imu_pub =
    n.advertise<sensor_msgs::Imu>("imu", 1000);
  sensor_msgs::Imu imu_msg;
  ros1_ign_bridge::testing::createTestMsg(imu_msg);

  // sensor_msgs::LaserScan.
  ros::Publisher laserscan_pub =
    n.advertise<sensor_msgs::LaserScan>("laserscan", 1000);
  sensor_msgs::LaserScan laserscan_msg;
  ros1_ign_bridge::testing::createTestMsg(laserscan_msg);

  // sensor_msgs::MagneticField.
  ros::Publisher magnetic_pub =
    n.advertise<sensor_msgs::MagneticField>("magnetic", 1000);
  sensor_msgs::MagneticField magnetic_msg;
  ros1_ign_bridge::testing::createTestMsg(magnetic_msg);

  while (ros::ok())
  {
    // Publish all messages.
    header_pub.publish(header_msg);
    string_pub.publish(string_msg);
    quaternion_pub.publish(quaternion_msg);
    vector3_pub.publish(vector3_msg);
    point_pub.publish(point_msg);
    pose_pub.publish(pose_msg);
    pose_stamped_pub.publish(pose_stamped_msg);
    transform_pub.publish(transform_msg);
    transform_stamped_pub.publish(transform_stamped_msg);
    image_pub.publish(image_msg);
    imu_pub.publish(imu_msg);
    laserscan_pub.publish(laserscan_msg);
    magnetic_pub.publish(magnetic_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
