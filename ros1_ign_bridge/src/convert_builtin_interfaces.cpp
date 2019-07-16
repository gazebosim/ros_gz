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

#include "ros1_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros1_ign_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(const std::string &input,
                              const std::string &old_delim,
                              const std::string new_delim)
{
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size())
  {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos)
    {
      output += new_delim;
      pos += old_delim.size();
    }

    last_pos = pos;
  }

  return output;
}

// Frame id from ROS to ign is not supported right now
// std::string frame_id_1_to_ign(const std::string &frame_id)
// {
//   return replace_delimiter(frame_id, "/", "::");
// }

std::string frame_id_ign_to_1(const std::string &frame_id)
{
  return replace_delimiter(frame_id, "::", "/");
}

template<>
void
convert_1_to_ign(
  const std_msgs::Float32 & ros1_msg,
  ignition::msgs::Float & ign_msg)
{
  ign_msg.set_data(ros1_msg.data);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros1_msg)
{
  ros1_msg.data = ign_msg.data();
}

template<>
void
convert_1_to_ign(
  const std_msgs::Header & ros1_msg,
  ignition::msgs::Header & ign_msg)
{
  ign_msg.mutable_stamp()->set_sec(ros1_msg.stamp.sec);
  ign_msg.mutable_stamp()->set_nsec(ros1_msg.stamp.nsec);
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
      ros1_msg.frame_id = frame_id_ign_to_1(aPair.value(0));
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
convert_ign_to_1(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros1_msg)
{
  ros1_msg.clock = ros::Time(ign_msg.sim().sec(), ign_msg.sim().nsec());
}

template<>
void
convert_1_to_ign(
  const rosgraph_msgs::Clock & ros1_msg,
  ignition::msgs::Clock & ign_msg)
{
  ign_msg.mutable_sim()->set_sec(ros1_msg.clock.sec);
  ign_msg.mutable_sim()->set_nsec(ros1_msg.clock.nsec);
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
  const geometry_msgs::Point & ros1_msg,
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
  geometry_msgs::Point & ros1_msg)
{
  ros1_msg.x = ign_msg.x();
  ros1_msg.y = ign_msg.y();
  ros1_msg.z = ign_msg.z();
}

template<>
void
convert_1_to_ign(
  const geometry_msgs::Pose & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_1_to_ign(ros1_msg.position, *ign_msg.mutable_position());
  convert_1_to_ign(ros1_msg.orientation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros1_msg)
{
  convert_ign_to_1(ign_msg.position(), ros1_msg.position);
  convert_ign_to_1(ign_msg.orientation(), ros1_msg.orientation);
}

template<>
void
convert_1_to_ign(
  const geometry_msgs::PoseStamped & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  convert_1_to_ign(ros1_msg.pose, ign_msg);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  convert_ign_to_1(ign_msg, ros1_msg.pose);
}

template<>
void
convert_1_to_ign(
  const geometry_msgs::Transform & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_1_to_ign(ros1_msg.translation , *ign_msg.mutable_position());
  convert_1_to_ign(ros1_msg.rotation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros1_msg)
{
  convert_ign_to_1(ign_msg.position(), ros1_msg.translation);
  convert_ign_to_1(ign_msg.orientation(), ros1_msg.rotation);
}

template<>
void
convert_1_to_ign(
  const geometry_msgs::TransformStamped & ros1_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  convert_1_to_ign(ros1_msg.transform, ign_msg);

  auto newPair = ign_msg.mutable_header()->add_data();
  newPair->set_key("child_frame_id");
  newPair->add_value(ros1_msg.child_frame_id);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  convert_ign_to_1(ign_msg, ros1_msg.transform);
  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros1_msg.child_frame_id = frame_id_ign_to_1(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_1_to_ign(
  const geometry_msgs::Twist & ros1_msg,
  ignition::msgs::Twist & ign_msg)
{
  convert_1_to_ign(ros1_msg.linear,  (*ign_msg.mutable_linear()));
  convert_1_to_ign(ros1_msg.angular, (*ign_msg.mutable_angular()));
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros1_msg)
{
  convert_ign_to_1(ign_msg.linear(), ros1_msg.linear);
  convert_ign_to_1(ign_msg.angular(), ros1_msg.angular);
}

template<>
void
convert_1_to_ign(
  const mav_msgs::Actuators & ros1_msg,
  ignition::msgs::Actuators & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  for (auto i = 0u; i < ros1_msg.angles.size(); ++i)
    ign_msg.add_position(ros1_msg.angles[i]);
  for (auto i = 0u; i < ros1_msg.angular_velocities.size(); ++i)
    ign_msg.add_velocity(ros1_msg.angular_velocities[i]);
  for (auto i = 0u; i < ros1_msg.normalized.size(); ++i)
    ign_msg.add_normalized(ros1_msg.normalized[i]);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Actuators & ign_msg,
  mav_msgs::Actuators & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  for (auto i = 0; i < ign_msg.position_size(); ++i)
    ros1_msg.angles.push_back(ign_msg.position(i));
  for (auto i = 0; i < ign_msg.velocity_size(); ++i)
    ros1_msg.angular_velocities.push_back(ign_msg.velocity(i));
  for (auto i = 0; i < ign_msg.normalized_size(); ++i)
    ros1_msg.normalized.push_back(ign_msg.normalized(i));
}

template<>
void
convert_1_to_ign(
  const nav_msgs::Odometry & ros1_msg,
  ignition::msgs::Odometry & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  convert_1_to_ign(ros1_msg.pose.pose, (*ign_msg.mutable_pose()));
  convert_1_to_ign(ros1_msg.twist.twist, (*ign_msg.mutable_twist()));

  auto childFrame = ign_msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(ros1_msg.child_frame_id);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  convert_ign_to_1(ign_msg.pose(), ros1_msg.pose.pose);
  convert_ign_to_1(ign_msg.twist(), ros1_msg.twist.twist);

  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros1_msg.child_frame_id = frame_id_ign_to_1(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_1_to_ign(
  const sensor_msgs::FluidPressure & ros1_msg,
  ignition::msgs::FluidPressure & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));
  ign_msg.set_pressure(ros1_msg.fluid_pressure);
  ign_msg.set_variance(ros1_msg.variance);
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::FluidPressure & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);
  ros1_msg.fluid_pressure = ign_msg.pressure();
  ros1_msg.variance = ign_msg.variance();
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
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::L_INT8);
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "mono16")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::L_INT16);
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ros1_msg.encoding == "rgb8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::RGB_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "rgba8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::RGBA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "bgra8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::BGRA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "rgb16")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::RGB_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros1_msg.encoding == "bgr8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::BGR_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros1_msg.encoding == "bgr16")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::BGR_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros1_msg.encoding == "32FC1")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::R_FLOAT32);
    num_channels = 1;
    octets_per_channel = 4u;
  }
  else
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT);
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

  if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::L_INT8)
  {
    ros1_msg.encoding = "mono8";
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::L_INT16)
  {
    ros1_msg.encoding = "mono16";
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::RGB_INT8)
  {
    ros1_msg.encoding = "rgb8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::RGBA_INT8)
  {
    ros1_msg.encoding = "rgba8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::BGRA_INT8)
  {
    ros1_msg.encoding = "bgra8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::RGB_INT16)
  {
    ros1_msg.encoding = "rgb16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::BGR_INT8)
  {
    ros1_msg.encoding = "bgr8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::BGR_INT16)
  {
    ros1_msg.encoding = "bgr16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::R_FLOAT32)
  {
    ros1_msg.encoding = "32FC1";
    num_channels = 1;
    octets_per_channel = 4u;
  }
  else
  {
    std::cerr << "Unsupported pixel format [" << ign_msg.pixel_format_type() << "]"
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
  const sensor_msgs::CameraInfo & ros1_msg,
  ignition::msgs::CameraInfo & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_width(ros1_msg.width);
  ign_msg.set_height(ros1_msg.height);

  auto distortion = ign_msg.mutable_distortion();
  if (ros1_msg.distortion_model == "plumb_bob")
  {
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::PLUMB_BOB);
  }
  else if (ros1_msg.distortion_model == "rational_polynomial")
  {
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL);
  }
  else if (ros1_msg.distortion_model == "equidistant")
  {
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::EQUIDISTANT);
  }
  else
  {
    std::cerr << "Unsupported distortion model [" << ros1_msg.distortion_model << "]"
              << std::endl;
  }
  for (auto i = 0u; i < ros1_msg.D.size(); ++i)
  {
    distortion->add_k(ros1_msg.D[i]);
  }

  auto intrinsics = ign_msg.mutable_intrinsics();
  for (auto i = 0u; i < ros1_msg.K.size(); ++i)
  {
    intrinsics->add_k(ros1_msg.K[i]);
  }

  auto projection = ign_msg.mutable_projection();
  for (auto i = 0u; i < ros1_msg.P.size(); ++i)
  {
    projection->add_p(ros1_msg.P[i]);
  }

  for (auto i = 0u; i < ros1_msg.R.size(); ++i)
  {
    ign_msg.add_rectification_matrix(ros1_msg.R[i]);
  }
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::CameraInfo & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  ros1_msg.height = ign_msg.height();
  ros1_msg.width = ign_msg.width();

  auto distortion = ign_msg.distortion();
  if (ign_msg.has_distortion())
  {
    auto distortion = ign_msg.distortion();
    if (distortion.model() ==
        ignition::msgs::CameraInfo::Distortion::PLUMB_BOB)
    {
      ros1_msg.distortion_model = "plumb_bob";
    }
    else if (distortion.model() ==
        ignition::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL)
    {
      ros1_msg.distortion_model = "rational_polynomial";
    }
    else if (distortion.model() ==
        ignition::msgs::CameraInfo::Distortion::EQUIDISTANT)
    {
      ros1_msg.distortion_model = "equidistant";
    }
    else
    {
      std::cerr << "Unsupported distortion model ["
                << distortion.model() << "]" << std::endl;
    }

    ros1_msg.D.resize(distortion.k_size());
    for (auto i = 0; i < distortion.k_size(); ++i)
    {
      ros1_msg.D[i] = distortion.k(i);
    }
  }

  auto intrinsics = ign_msg.intrinsics();
  if (ign_msg.has_intrinsics())
  {
    auto intrinsics = ign_msg.intrinsics();

    for (auto i = 0; i < intrinsics.k_size(); ++i)
    {
      ros1_msg.K[i] = intrinsics.k(i);
    }
  }

  auto projection = ign_msg.projection();
  if (ign_msg.has_projection())
  {
    auto projection = ign_msg.projection();

    for (auto i = 0; i < projection.p_size(); ++i)
    {
      ros1_msg.P[i] = projection.p(i);
    }
  }

  for (auto i = 0; i < ign_msg.rectification_matrix_size(); ++i)
  {
    ros1_msg.R[i] = ign_msg.rectification_matrix(i);
  }
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
  const sensor_msgs::JointState & ros1_msg,
  ignition::msgs::Model & ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  for (auto i = 0u; i < ros1_msg.position.size(); ++i)
  {
    auto newJoint = ign_msg.add_joint();
    newJoint->set_name(ros1_msg.name[i]);
    newJoint->mutable_axis1()->set_position(ros1_msg.position[i]);
    newJoint->mutable_axis1()->set_velocity(ros1_msg.velocity[i]);
    newJoint->mutable_axis1()->set_force(ros1_msg.effort[i]);
  }
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::JointState & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  for (auto i = 0; i < ign_msg.joint_size(); ++i)
  {
    ros1_msg.name.push_back(ign_msg.joint(i).name());
    ros1_msg.position.push_back(ign_msg.joint(i).axis1().position());
    ros1_msg.velocity.push_back(ign_msg.joint(i).axis1().velocity());
    ros1_msg.effort.push_back(ign_msg.joint(i).axis1().force());
  }
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
  ros1_msg.header.frame_id = frame_id_ign_to_1(ign_msg.frame());

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
  ignition::msgs::PointCloudPacked &ign_msg)
{
  convert_1_to_ign(ros1_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_height(ros1_msg.height);
  ign_msg.set_width(ros1_msg.width);
  ign_msg.set_is_bigendian(ros1_msg.is_bigendian);
  ign_msg.set_point_step(ros1_msg.point_step);
  ign_msg.set_row_step(ros1_msg.row_step);
  ign_msg.set_is_dense(ros1_msg.is_dense);
  ign_msg.mutable_data()->resize(ros1_msg.data.size());
  memcpy(ign_msg.mutable_data()->data(), ros1_msg.data.data(),
         ros1_msg.data.size());

  for (unsigned int i = 0; i < ros1_msg.fields.size(); ++i)
  {
    ignition::msgs::PointCloudPacked::Field *pf = ign_msg.add_field();
    pf->set_name(ros1_msg.fields[i].name);
    pf->set_count(ros1_msg.fields[i].count);
    pf->set_offset(ros1_msg.fields[i].offset);
    switch (ros1_msg.fields[i].datatype)
    {
      default:
      case sensor_msgs::PointField::INT8:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::INT8);
        break;
      case sensor_msgs::PointField::UINT8:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::UINT8);
        break;
      case sensor_msgs::PointField::INT16:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::INT16);
        break;
      case sensor_msgs::PointField::UINT16:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::UINT16);
        break;
      case sensor_msgs::PointField::INT32:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::INT32);
        break;
      case sensor_msgs::PointField::UINT32:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::UINT32);
        break;
      case sensor_msgs::PointField::FLOAT32:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::FLOAT32);
        break;
      case sensor_msgs::PointField::FLOAT64:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::FLOAT64);
        break;
    };
  }
}

template<>
void
convert_ign_to_1(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::PointCloud2 & ros1_msg)
{
  convert_ign_to_1(ign_msg.header(), ros1_msg.header);

  ros1_msg.height = ign_msg.height();
  ros1_msg.width = ign_msg.width();
  ros1_msg.is_bigendian = ign_msg.is_bigendian();
  ros1_msg.point_step = ign_msg.point_step();
  ros1_msg.row_step = ign_msg.row_step();
  ros1_msg.is_dense = ign_msg.is_dense();
  ros1_msg.data.resize(ign_msg.data().size());
  memcpy(ros1_msg.data.data(), ign_msg.data().c_str(), ign_msg.data().size());

  for (int i = 0; i < ign_msg.field_size(); ++i)
  {
    sensor_msgs::PointField pf;
    pf.name = ign_msg.field(i).name();
    pf.count = ign_msg.field(i).count();
    pf.offset = ign_msg.field(i).offset();
    switch (ign_msg.field(i).datatype())
    {
      default:
      case ignition::msgs::PointCloudPacked::Field::INT8:
        pf.datatype = sensor_msgs::PointField::INT8;
        break;
      case ignition::msgs::PointCloudPacked::Field::UINT8:
        pf.datatype = sensor_msgs::PointField::UINT8;
        break;
      case ignition::msgs::PointCloudPacked::Field::INT16:
        pf.datatype = sensor_msgs::PointField::INT16;
        break;
      case ignition::msgs::PointCloudPacked::Field::UINT16:
        pf.datatype = sensor_msgs::PointField::UINT16;
        break;
      case ignition::msgs::PointCloudPacked::Field::INT32:
        pf.datatype = sensor_msgs::PointField::INT32;
        break;
      case ignition::msgs::PointCloudPacked::Field::UINT32:
        pf.datatype = sensor_msgs::PointField::UINT32;
        break;
      case ignition::msgs::PointCloudPacked::Field::FLOAT32:
        pf.datatype = sensor_msgs::PointField::FLOAT32;
        break;
      case ignition::msgs::PointCloudPacked::Field::FLOAT64:
        pf.datatype = sensor_msgs::PointField::FLOAT64;
        break;
    };
    ros1_msg.fields.push_back(pf);
  }
}

}  // namespace ros1_ign_bridge
