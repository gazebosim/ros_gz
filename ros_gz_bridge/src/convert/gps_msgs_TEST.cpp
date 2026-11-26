// Copyright 2026 Open Source Robotics Foundation, Inc.
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

// Tests for gps_msgs <-> gz::msgs converters.
//
// gps_msgs/GPSFix.track is in degrees clockwise from North (0 = North,
// 90 = East), while gz::msgs::NavSat carries East/North/Up velocities.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <ros_gz_bridge/convert/gps_msgs.hpp>

namespace
{
constexpr double kTol = 1e-9;

struct CompassCase
{
  double east;
  double north;
  double track_deg;
};

// Speed 3 m/s along each compass and intercardinal direction.
const double kDiag = 3.0 / std::sqrt(2.0);
const CompassCase kCases[] = {
  {0.0, 3.0, 0.0},  // North
  {kDiag, kDiag, 45.0},  // North-East
  {3.0, 0.0, 90.0},  // East
  {kDiag, -kDiag, 135.0},  // South-East
  {0.0, -3.0, 180.0},  // South
  {-kDiag, -kDiag, 225.0},  // South-West
  {-3.0, 0.0, 270.0},  // West
  {-kDiag, kDiag, 315.0},  // North-West
};
}  // namespace

TEST(GPSFixGzToRos, CompassPoints)
{
  for (const auto & c : kCases) {
    SCOPED_TRACE("track " + std::to_string(c.track_deg));
    gz::msgs::NavSat gz_msg;
    gz_msg.set_velocity_east(c.east);
    gz_msg.set_velocity_north(c.north);
    gz_msg.set_velocity_up(0.5);

    gps_msgs::msg::GPSFix ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);

    EXPECT_NEAR(3.0, ros_msg.speed, kTol);
    EXPECT_NEAR(c.track_deg, ros_msg.track, kTol);
    EXPECT_GE(ros_msg.track, 0.0);
    EXPECT_LT(ros_msg.track, 360.0);
    EXPECT_DOUBLE_EQ(0.5, ros_msg.climb);
  }
}

TEST(GPSFixRosToGz, CompassPoints)
{
  for (const auto & c : kCases) {
    SCOPED_TRACE("track " + std::to_string(c.track_deg));
    gps_msgs::msg::GPSFix ros_msg;
    ros_msg.speed = 3.0;
    ros_msg.track = c.track_deg;
    ros_msg.climb = -0.5;

    gz::msgs::NavSat gz_msg;
    ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);

    EXPECT_NEAR(c.east, gz_msg.velocity_east(), kTol);
    EXPECT_NEAR(c.north, gz_msg.velocity_north(), kTol);
    EXPECT_DOUBLE_EQ(-0.5, gz_msg.velocity_up());
  }
}

TEST(GPSFix, GzRosGzRoundTrip)
{
  const double velocities[][2] = {
    {0.0, 3.0}, {3.0, 0.0}, {0.0, -3.0}, {-3.0, 0.0},
    {1.2, -4.7}, {-2.5, 0.3}, {0.001, 0.002},
  };
  for (const auto & v : velocities) {
    gz::msgs::NavSat in;
    in.set_velocity_east(v[0]);
    in.set_velocity_north(v[1]);
    in.set_velocity_up(1.5);

    gps_msgs::msg::GPSFix ros_msg;
    ros_gz_bridge::convert_gz_to_ros(in, ros_msg);
    gz::msgs::NavSat out;
    ros_gz_bridge::convert_ros_to_gz(ros_msg, out);

    EXPECT_NEAR(v[0], out.velocity_east(), kTol);
    EXPECT_NEAR(v[1], out.velocity_north(), kTol);
    EXPECT_DOUBLE_EQ(1.5, out.velocity_up());
  }
}

TEST(GPSFix, RosGzRosRoundTrip)
{
  for (double track = 0.0; track < 360.0; track += 15.0) {
    gps_msgs::msg::GPSFix in;
    in.speed = 2.0;
    in.track = track;

    gz::msgs::NavSat gz_msg;
    ros_gz_bridge::convert_ros_to_gz(in, gz_msg);
    gps_msgs::msg::GPSFix out;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, out);

    EXPECT_NEAR(2.0, out.speed, kTol);
    EXPECT_NEAR(track, out.track, 1e-6);
  }
}

TEST(GPSFix, Stationary)
{
  gz::msgs::NavSat gz_msg;
  gps_msgs::msg::GPSFix ros_msg;
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);

  EXPECT_DOUBLE_EQ(0.0, ros_msg.speed);
  EXPECT_DOUBLE_EQ(0.0, ros_msg.track);
  EXPECT_FALSE(std::isnan(ros_msg.track));

  gz::msgs::NavSat back;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, back);
  EXPECT_DOUBLE_EQ(0.0, back.velocity_east());
  EXPECT_DOUBLE_EQ(0.0, back.velocity_north());
}
