// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <std_msgs/String.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ros1_ign_bridge_test_client");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  std_msgs::String msg;
  msg.data = "hello";

  while (ros::ok())
  {
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
