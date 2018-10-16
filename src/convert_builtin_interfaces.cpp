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

#include <algorithm>
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
  const geometry_msgs::Quaternion & ros1_msg,
  ignition::msgs::Quaternion & ign_msg)
{
  ign_msg.set_x(ros1_msg.x);
  ign_msg.set_y(ros1_msg.y);
  ign_msg.set_z(ros1_msg.z);
  ign_msg.set_w(ros1_msg.w);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros1_msg)
{
  ros1_msg.x = ign_msg.x();
  ros1_msg.y = ign_msg.y();
  ros1_msg.z = ign_msg.z();
  ros1_msg.w = ign_msg.w();
}

template<>
void
convert_1_to_ign(
  const geometry_msgs::Vector3 & ros1_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros1_msg.x);
  ign_msg.set_y(ros1_msg.y);
  ign_msg.set_z(ros1_msg.z);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros1_msg)
{
  ros1_msg.x = ign_msg.x();
  ros1_msg.y = ign_msg.y();
  ros1_msg.z = ign_msg.z();
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::FluidPressure & ros1_msg,
  ignition::msgs::Fluid & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  std::cerr << "Unsupported conversion from [sensor_msgs::FluidPressure] to "
            << "[ignition::msgs::Fluid]" << std::endl;

  // This might be relevant:
  // https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_pressure_plugin.cpp#L97
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Fluid & ign_msg,
  sensor_msgs::FluidPressure & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  // variance isn't supported in ignition::msgs::Fluid

  std::cerr << "Unsupported conversion from [ignition::msgs::Fluid] to "
            << "[sensor_msgs::FluidPressure]" << std::endl;

  // This might be relevant:
  // https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_pressure_plugin.cpp#L97
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
  unsigned int octets_per_channel;

  if (ros1_msg.encoding == "mono8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::L_INT8);
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "mono16")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::L_INT16);
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ros1_msg.encoding == "rgb8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::RGB_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "rgba8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::RGBA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "bgra8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::BGRA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "rgb16")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::RGB_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros1_msg.encoding == "bgr8")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::BGR_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "bgr16")
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::BGR_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else
  {
    ign_msg.set_pixel_format(
      ignition::common::Image::PixelFormatType::UNKNOWN_PIXEL_FORMAT);
    std::cerr << "Unsupported pixel format [" << ros1_msg.encoding << "]"
              << std::endl;
    return;
  }

  ign_msg.set_step(ign_msg.width() * num_channels * octets_per_channel);

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
  unsigned int octets_per_channel;

  if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::L_INT8)
  {
    ros1_msg.encoding = "mono8";
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::L_INT16)
  {
    ros1_msg.encoding = "mono16";
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::RGB_INT8)
  {
    ros1_msg.encoding = "rgb8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::RGBA_INT8)
  {
    ros1_msg.encoding = "rgba8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::BGRA_INT8)
  {
    ros1_msg.encoding = "bgra8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::RGB_INT16)
  {
    ros1_msg.encoding = "rgb16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::BGR_INT8)
  {
    ros1_msg.encoding = "bgr8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format() ==
      ignition::common::Image::PixelFormatType::BGR_INT16)
  {
    ros1_msg.encoding = "bgr16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else
  {
    std::cerr << "Unsupported pixel format [" << ign_msg.pixel_format() << "]"
              << std::endl;
    return;
  }

  ros1_msg.is_bigendian = false;
  ros1_msg.step = ros1_msg.width * num_channels * octets_per_channel;

  auto count = ros1_msg.step * ros1_msg.height;
  ros1_msg.data.resize(ros1_msg.step * ros1_msg.height);
  std::copy(
    ign_msg.data().begin(),
    ign_msg.data().begin() + count,
    ros1_msg.data.begin());
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::Imu & ros1_msg,
  ignition::msgs::IMU & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  // ToDo: Verify that this is the expected value (probably not).
  ign_msg.set_entity_name(ros1_msg.header.frame_id);

  convert_1_to_ign(ros1_msg.orientation, (*ign_msg.mutable_orientation()));
  convert_1_to_ign(ros1_msg.angular_velocity,
                   (*ign_msg.mutable_angular_velocity()));
  convert_1_to_ign(ros1_msg.linear_acceleration,
                   (*ign_msg.mutable_linear_acceleration()));
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  convert_ign_to_1(ign_msg.orientation(), ros1_msg.orientation);
  convert_ign_to_1(ign_msg.angular_velocity(), ros1_msg.angular_velocity);
  convert_ign_to_1(ign_msg.linear_acceleration(), ros1_msg.linear_acceleration);

  // Covariances not supported in Ignition::msgs::IMU
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::LaserScan & ros1_msg,
  ignition::msgs::LaserScan & ign_msg)
{
  const unsigned int num_readings =
    (ros1_msg.angle_max - ros1_msg.angle_min) / ros1_msg.angle_increment;

  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  ign_msg.set_frame(ros1_msg.header.frame_id);
  ign_msg.set_angle_min(ros1_msg.angle_min);
  ign_msg.set_angle_max(ros1_msg.angle_max);
  ign_msg.set_angle_step(ros1_msg.angle_increment);
  ign_msg.set_range_min(ros1_msg.range_min);
  ign_msg.set_range_max(ros1_msg.range_max);
  ign_msg.set_count(num_readings);

  // Not supported in sensor_msgs::LaserScan.
  ign_msg.set_vertical_angle_min(0.0);
  ign_msg.set_vertical_angle_max(0.0);
  ign_msg.set_vertical_angle_step(0.0);
  ign_msg.set_vertical_count(0u);

  for (auto i = 0u; i < ign_msg.count(); ++i)
  {
    ign_msg.add_ranges(ros1_msg.ranges[i]);
    ign_msg.add_intensities(ros1_msg.intensities[i]);
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

  auto count = ign_msg.count();
  auto vertical_count = ign_msg.vertical_count();

  // If there are multiple vertical beams, use the one in the middle.
  size_t start = (vertical_count / 2) * count;

  // Copy ranges into ROS message.
  ros1_msg.ranges.resize(count);
  std::copy(
    ign_msg.ranges().begin() + start,
    ign_msg.ranges().begin() + start + count,
    ros1_msg.ranges.begin());

  // Copy intensities into ROS message.
  ros1_msg.intensities.resize(count);
  std::copy(
    ign_msg.intensities().begin() + start,
    ign_msg.intensities().begin() + start + count,
    ros1_msg.intensities.begin());
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::MagneticField & ros1_msg,
  ignition::msgs::Magnetometer & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  convert_1_to_ign(ros1_msg.magnetic_field, (*ign_msg.mutable_field_tesla()));
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  convert_ign_to_1(ign_msg.field_tesla(), ros1_msg.magnetic_field);

  // magnetic_field_covariance is not supported in Ignition::Msgs::Magnetometer.
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
