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

#include <exception>

#include "ros1_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros1_ign_bridge
{

template<>
void
convert_1_to_ign(
  const std_msgs::Header & ros1_type,
  ignition::msgs::Header & ign_msg)
{
  ign_msg.mutable_stamp()->set_sec(ros1_type.stamp.toSec());
  ign_msg.mutable_stamp()->set_nsec(ros1_type.stamp.toNSec());
  auto newPair = ign_msg.add_data();
  newPair->set_key("seq");
  newPair->add_value(std::to_string(ros1_type.seq));
  newPair = ign_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros1_type.frame_id);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros1_type)
{
  ros1_type.stamp = ros::Time(ign_msg.stamp().sec(), ign_msg.stamp().nsec());
  for (auto i = 0; i < ign_msg.data_size(); ++i)
  {
    auto aPair = ign_msg.data(i);
    if (aPair.key() == "seq" && aPair.value_size() > 0)
    {
      std::string value = aPair.value(0);
      try
      {
        unsigned long ul = std::stoul(value, nullptr);
        ros1_type.seq = ul;
      }
      catch (std::exception & e)
      {
        std::cerr << "Exception converting [" << value << "] to an "
                  << "unsigned int" << std::endl;
      }
    }
    else if (aPair.key() == "frame_id" && aPair.value_size() > 0)
    {
      ros1_type.frame_id = aPair.value(0);
    }
  }
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::LaserScan & ros1_type,
  ignition::msgs::LaserScan & ign_msg)
{
  convert_1_to_ign(ros1_type.header, (*ign_msg.mutable_header()));
  ign_msg.set_frame(ros1_type.header.frame_id);
  ign_msg.set_angle_min(ros1_type.angle_min);
  ign_msg.set_angle_max(ros1_type.angle_max);
  ign_msg.set_angle_step(ros1_type.angle_increment);
  ign_msg.set_range_min(ros1_type.range_min);
  ign_msg.set_range_max(ros1_type.range_max);
  ign_msg.set_count(sizeof(ros1_type.ranges));

  // Not supported in sensor_msgs::LaserScan.
  ign_msg.set_vertical_angle_min(0.0);
  ign_msg.set_vertical_angle_max(0.0);
  ign_msg.set_vertical_angle_step(0.0);
  ign_msg.set_vertical_count(0u);

  for (auto i = 0u; i < ign_msg.count(); ++i)
  {
    ign_msg.add_ranges(ros1_type.ranges[i]);
    ign_msg.add_ranges(ros1_type.intensities[i]);
  }
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros1_type)
{
  convert_ign_to_1(ign_msg.header(), ros1_type.header);
  ros1_type.header.frame_id = ign_msg.frame();

  ros1_type.angle_min = ign_msg.angle_min();
  ros1_type.angle_max = ign_msg.angle_max();
  ros1_type.angle_increment = ign_msg.angle_step();

  // Not supported in ignition::msgs::LaserScan.
  ros1_type.time_increment = 0.0;
  ros1_type.scan_time = 0.0;

  ros1_type.range_min = ign_msg.range_min();
  ros1_type.range_max = ign_msg.range_max();

  unsigned int num_readings = ign_msg.count();
  ros1_type.ranges.resize(num_readings);
  ros1_type.intensities.resize(num_readings);

  for (auto i = 0u; i < num_readings; ++i)
  {
    ros1_type.ranges[i] = ign_msg.ranges(i);
    ros1_type.intensities[i] = ign_msg.intensities(i);
  }
}

template<>
void
convert_1_to_ign(
  const std_msgs::String & ros1_type,
  ignition::msgs::StringMsg & ign_msg)
{
  ign_msg.set_data(ros1_type.data);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros1_type)
{
  ros1_type.data = ign_msg.data();
}

}  // namespace ros1_ign_bridge
