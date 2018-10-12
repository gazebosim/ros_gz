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
#include <ignition/common/Image.hh>

#include "ros1_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros1_ign_bridge
{

template<>
void
convert_1_to_ign(
  const std_msgs::Header & ros1_msg,
  ignition::msgs::Header & ign_msg)
{
  ign_msg.mutable_stamp()->set_sec(ros1_msg.stamp.toSec());
  ign_msg.mutable_stamp()->set_nsec(ros1_msg.stamp.toNSec());
  auto newPair = ign_msg.add_data();
  newPair->set_key("seq");
  newPair->add_value(std::to_string(ros1_msg.seq));
  newPair = ign_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros1_msg.frame_id);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros1_msg)
{
  ros1_msg.stamp = ros::Time(ign_msg.stamp().sec(), ign_msg.stamp().nsec());
  for (auto i = 0; i < ign_msg.data_size(); ++i)
  {
    auto aPair = ign_msg.data(i);
    if (aPair.key() == "seq" && aPair.value_size() > 0)
    {
      std::string value = aPair.value(0);
      try
      {
        unsigned long ul = std::stoul(value, nullptr);
        ros1_msg.seq = ul;
      }
      catch (std::exception & e)
      {
        std::cerr << "Exception converting [" << value << "] to an "
                  << "unsigned int" << std::endl;
      }
    }
    else if (aPair.key() == "frame_id" && aPair.value_size() > 0)
    {
      ros1_msg.frame_id = aPair.value(0);
    }
  }
}

template<>
void
convert_1_to_ign(
  const std_msgs::String & ros1_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ign_msg.set_data(ros1_msg.data);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros1_msg)
{
  ros1_msg.data = ign_msg.data();
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::Image & ros1_msg,
  ignition::msgs::Image & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_width(ros1_msg.width);
  ign_msg.set_height(ros1_msg.height);

  unsigned int num_channels;
  unsigned int size_per_channel;

  if (ros1_msg.encoding == "mono8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::L_INT8);
    num_channels = 1;
    size_per_channel = 8u;
  }
  else if (ros1_msg.encoding == "mono16")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::L_INT16);
    num_channels = 1;
    size_per_channel = 16u;
  }
  else if (ros1_msg.encoding == "rgb8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::RGB_INT8);
    num_channels = 3;
    size_per_channel = 8u;
  }
  else if (ros1_msg.encoding == "rgba8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::RGBA_INT8);
    num_channels = 4;
    size_per_channel = 8u;
  }
  else if (ros1_msg.encoding == "bgra8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::BGRA_INT8);
    num_channels = 4;
    size_per_channel = 8u;
  }
  else if (ros1_msg.encoding == "rgb16")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::RGB_INT16);
    num_channels = 3;
    size_per_channel = 16u;
  }
  else if (ros1_msg.encoding == "bgr8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::BGR_INT8);
    num_channels = 3;
    size_per_channel = 8u;
  }
  else if (ros1_msg.encoding == "bgr16")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::BGR_INT16);
    num_channels = 3;
    size_per_channel = 16u;
  }
  else
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::UNKNOWN_PIXEL_FORMAT);
    std::cerr << "Unsupported pixel format [" << ros1_msg.encoding << "]"
              << std::endl;
    return;
  }

  ign_msg.set_step(ign_msg.width() * num_channels * size_per_channel);

  ign_msg.set_data(&(ros1_msg.data[0]), ign_msg.step() * ign_msg.height());
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::Image & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  ros1_msg.height = ign_msg.height();
  ros1_msg.width = ign_msg.width();

  unsigned int num_channels;
  unsigned int size_per_channel;

  if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::L_INT8)
  {
    ros1_msg.encoding = "mono8";
    num_channels = 1;
    size_per_channel = 8u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::L_INT16)
  {
    ros1_msg.encoding = "mono16";
    num_channels = 1;
    size_per_channel = 16u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::RGB_INT8)
  {
    ros1_msg.encoding = "rgb8";
    num_channels = 3;
    size_per_channel = 8u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::RGBA_INT8)
  {
    ros1_msg.encoding = "rgba8";
    num_channels = 4;
    size_per_channel = 8u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::BGRA_INT8)
  {
    ros1_msg.encoding = "bgra8";
    num_channels = 4;
    size_per_channel = 8u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::RGB_INT16)
  {
    ros1_msg.encoding = "rgb16";
    num_channels = 3;
    size_per_channel = 16u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::BGR_INT8)
  {
    ros1_msg.encoding = "bgr8";
    num_channels = 3;
    size_per_channel = 8u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::BGR_INT16)
  {
    ros1_msg.encoding = "bgr16";
    num_channels = 3;
    size_per_channel = 16u;
  }
  else
  {
    std::cerr << "Unsupported pixel format [" << ign_msg.pixel_format() << "]"
              << std::endl;
    return;
  }

  ros1_msg.is_bigendian = false;
  ros1_msg.step = ros1_msg.width * num_channels * size_per_channel;

  ros1_msg.data.resize(ros1_msg.step * ros1_msg.height);
  memcpy(&ros1_msg.data[0], ign_msg.data().c_str(),
         ros1_msg.step * ros1_msg.height);
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::LaserScan & ros1_msg,
  ignition::msgs::LaserScan & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  ign_msg.set_frame(ros1_msg.header.frame_id);
  ign_msg.set_angle_min(ros1_msg.angle_min);
  ign_msg.set_angle_max(ros1_msg.angle_max);
  ign_msg.set_angle_step(ros1_msg.angle_increment);
  ign_msg.set_range_min(ros1_msg.range_min);
  ign_msg.set_range_max(ros1_msg.range_max);
  ign_msg.set_count(sizeof(ros1_msg.ranges));

  // Not supported in sensor_msgs::LaserScan.
  ign_msg.set_vertical_angle_min(0.0);
  ign_msg.set_vertical_angle_max(0.0);
  ign_msg.set_vertical_angle_step(0.0);
  ign_msg.set_vertical_count(0u);

  for (auto i = 0u; i < ign_msg.count(); ++i)
  {
    ign_msg.add_ranges(ros1_msg.ranges[i]);
    ign_msg.add_ranges(ros1_msg.intensities[i]);
  }
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  ros1_msg.header.frame_id = ign_msg.frame();

  ros1_msg.angle_min = ign_msg.angle_min();
  ros1_msg.angle_max = ign_msg.angle_max();
  ros1_msg.angle_increment = ign_msg.angle_step();

  // Not supported in ignition::msgs::LaserScan.
  ros1_msg.time_increment = 0.0;
  ros1_msg.scan_time = 0.0;

  ros1_msg.range_min = ign_msg.range_min();
  ros1_msg.range_max = ign_msg.range_max();

  unsigned int num_readings = ign_msg.count();
  ros1_msg.ranges.resize(num_readings);
  ros1_msg.intensities.resize(num_readings);

  for (auto i = 0u; i < num_readings; ++i)
  {
    ros1_msg.ranges[i] = ign_msg.ranges(i);
    ros1_msg.intensities[i] = ign_msg.intensities(i);
  }
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::PointCloud2 & ros1_msg,
  ignition::msgs::PointCloud & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  std::cerr << "Unsupported conversion from [sensor_msgs::PointCloud2] to "
            << "[ignition::msgs::PointCloud]" << std::endl;
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::PointCloud & ign_msg,
  sensor_msgs::PointCloud2 & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  std::cerr << "Unsupported conversion from [ignition::msgs::PointCloud] to "
            << "[sensor_msgs::PointCloud2]" << std::endl;
}

}  // namespace ros1_ign_bridge
