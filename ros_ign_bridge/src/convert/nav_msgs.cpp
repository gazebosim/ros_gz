// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <exception>
#include <ros/console.h>

#include "ros_ign_bridge/convert.hpp"
#include "convert/frame_id.hpp"

namespace ros_ign_bridge
{

template<> void
convert_ros_to_ign(
  const nav_msgs::OccupancyGrid & ros_msg,
  ignition::msgs::OccupancyGrid & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  ign_msg.mutable_info()->mutable_map_load_time()->set_sec(
      ros_msg.info.map_load_time.sec);
  ign_msg.mutable_info()->mutable_map_load_time()->set_nsec(
      ros_msg.info.map_load_time.nsec);

  ign_msg.mutable_info()->set_resolution(
      ros_msg.info.resolution);
  ign_msg.mutable_info()->set_width(
      ros_msg.info.width);
  ign_msg.mutable_info()->set_height(
      ros_msg.info.height);

  convert_ros_to_ign(ros_msg.info.origin, 
      (*ign_msg.mutable_info()->mutable_origin()));

  ign_msg.set_data(&ros_msg.data[0], ros_msg.data.size());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::OccupancyGrid & ign_msg,
  nav_msgs::OccupancyGrid & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  ros_msg.info.map_load_time.sec = ign_msg.info().map_load_time().sec();
  ros_msg.info.map_load_time.nsec = ign_msg.info().map_load_time().nsec();
  ros_msg.info.resolution = ign_msg.info().resolution();
  ros_msg.info.width = ign_msg.info().width();
  ros_msg.info.height = ign_msg.info().height();

  convert_ign_to_ros(ign_msg.info().origin(), ros_msg.info.origin);

  ros_msg.data.resize(ign_msg.data().size());
  memcpy(&ros_msg.data[0], ign_msg.data().c_str(), ign_msg.data().size());
}

template<>
void
convert_ros_to_ign(
  const nav_msgs::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose.pose, (*ign_msg.mutable_pose()));
  convert_ros_to_ign(ros_msg.twist.twist, (*ign_msg.mutable_twist()));

  auto childFrame = ign_msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg.pose(), ros_msg.pose.pose);
  convert_ign_to_ros(ign_msg.twist(), ros_msg.twist.twist);

  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}


}  // namespace ros_ign_bridge

