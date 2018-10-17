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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>

//////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ros1_string_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  // std_msgs::Header.
  ros::Publisher header_pub = n.advertise<std_msgs::Header>("header", 1000);
  std_msgs::Header header_msg;
  header_msg.seq = 1;
  header_msg.stamp.sec = 2;
  header_msg.stamp.nsec = 3;
  header_msg.frame_id = "frame_id_value";

  // std_msgs::String.
  ros::Publisher string_pub = n.advertise<std_msgs::String>("string", 1000);
  std_msgs::String string_msg;
  string_msg.data = "string";

  // geometry_msgs::Quaternion.
  ros::Publisher quaternion_pub =
    n.advertise<geometry_msgs::Quaternion>("quaternion", 1000);
  geometry_msgs::Quaternion quaternion_msg;
  quaternion_msg.x = 1;
  quaternion_msg.y = 2;
  quaternion_msg.z = 3;
  quaternion_msg.w = 4;

  // geometry_msgs::Vector3.
  ros::Publisher vector3_pub =
    n.advertise<geometry_msgs::Vector3>("vector3", 1000);
  geometry_msgs::Vector3 vector3_msg;
  vector3_msg.x = 1;
  vector3_msg.y = 2;
  vector3_msg.z = 3;

  // sensor_msgs::Image.
  ros::Publisher image_pub =
    n.advertise<sensor_msgs::Image>("image", 1000);
  sensor_msgs::Image image_msg;
  image_msg.header = header_msg;
  image_msg.width = 320;
  image_msg.height = 240;
  image_msg.encoding = "rgb8";
  image_msg.is_bigendian = false;
  image_msg.step = image_msg.width * 3;
  image_msg.data.resize(image_msg.height * image_msg.step, 1);

  // sensor_msgs::Imu.
  ros::Publisher imu_pub =
    n.advertise<sensor_msgs::Imu>("imu", 1000);
  sensor_msgs::Imu imu_msg;
  imu_msg.header = header_msg;
  imu_msg.orientation = quaternion_msg;
  imu_msg.angular_velocity = vector3_msg;
  imu_msg.linear_acceleration = vector3_msg;

  // sensor_msgs::LaserScan.
  ros::Publisher laserscan_pub =
    n.advertise<sensor_msgs::LaserScan>("laserscan", 1000);
  const unsigned int num_readings = 100u;
  const double laser_frequency = 40;
  sensor_msgs::LaserScan laserscan_msg;
  laserscan_msg.header = header_msg;
  laserscan_msg.angle_min = -1.57;
  laserscan_msg.angle_max = 1.57;
  laserscan_msg.angle_increment = 3.14 / num_readings;
  laserscan_msg.time_increment = (1 / laser_frequency) / (num_readings);
  laserscan_msg.scan_time = 0;
  laserscan_msg.range_min = 1;
  laserscan_msg.range_max = 2;
  laserscan_msg.ranges.resize(num_readings, 0);
  laserscan_msg.intensities.resize(num_readings, 1);

  // sensor_msgs::MagneticField.
  ros::Publisher magnetic_pub =
    n.advertise<sensor_msgs::MagneticField>("magnetic", 1000);
  sensor_msgs::MagneticField magnetic_msg;
  magnetic_msg.header = header_msg;
  magnetic_msg.magnetic_field = vector3_msg;
  magnetic_msg.magnetic_field_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  while (ros::ok())
  {
    // Publish all messages.
    header_pub.publish(header_msg);
    string_pub.publish(string_msg);
    quaternion_pub.publish(quaternion_msg);
    vector3_pub.publish(vector3_msg);
    image_pub.publish(image_msg);
    imu_pub.publish(imu_msg);
    laserscan_pub.publish(laserscan_msg);
    magnetic_pub.publish(magnetic_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
