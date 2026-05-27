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

// Bounds-safety tests for sensor_msgs <-> gz::msgs converters.
//
// Every test in this file targets a specific out-of-bounds read or write
// reachable from a *spec-conformant* input message. Without the
// corresponding fixes these tests either crash (under AddressSanitizer)
// or produce garbage values.

#include <gtest/gtest.h>

#include <ros_gz_bridge/convert/sensor_msgs.hpp>

// ---------------------------------------------------------------------------
// JointState ROS->GZ : gh issue #118
//
// sensor_msgs/JointState allows `name`, `velocity`, and `effort` to be
// shorter than `position` (the spec explicitly says they may be empty).
// joint_state_publisher publishes `position`-only messages, which made
// the bridge crash with signal 245 (issue #118, open since 2020).
// ---------------------------------------------------------------------------
TEST(JointStateRosToGz, PositionOnlyDoesNotReadPastVelocityOrEffort)
{
  sensor_msgs::msg::JointState ros_msg;
  ros_msg.name = {"joint1", "joint2"};
  ros_msg.position = {0.1, 0.2};
  // velocity and effort intentionally empty.

  gz::msgs::Model gz_msg;
  ASSERT_NO_FATAL_FAILURE(ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg));

  ASSERT_EQ(2, gz_msg.joint_size());
  EXPECT_EQ("joint1", gz_msg.joint(0).name());
  EXPECT_DOUBLE_EQ(0.1, gz_msg.joint(0).axis1().position());
  EXPECT_DOUBLE_EQ(0.0, gz_msg.joint(0).axis1().velocity());
  EXPECT_DOUBLE_EQ(0.0, gz_msg.joint(0).axis1().force());
  EXPECT_EQ("joint2", gz_msg.joint(1).name());
  EXPECT_DOUBLE_EQ(0.2, gz_msg.joint(1).axis1().position());
}

TEST(JointStateRosToGz, NameShorterThanPositionDoesNotOverread)
{
  sensor_msgs::msg::JointState ros_msg;
  // Two positions but only one name — the bridge must not read name[1].
  ros_msg.name = {"only_joint"};
  ros_msg.position = {0.1, 0.2};

  gz::msgs::Model gz_msg;
  ASSERT_NO_FATAL_FAILURE(ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg));

  ASSERT_EQ(2, gz_msg.joint_size());
  EXPECT_EQ("only_joint", gz_msg.joint(0).name());
  EXPECT_EQ("", gz_msg.joint(1).name());
}

// ---------------------------------------------------------------------------
// LaserScan ROS->GZ : count is computed from angle range / increment,
// which can disagree with the actual array sizes due to FP rounding.
// Many lidars also publish no intensities at all. The bridge must clamp
// to the actual array sizes instead of trusting the computed count.
// ---------------------------------------------------------------------------
TEST(LaserScanRosToGz, ComputedCountLargerThanRangesArrayIsClamped)
{
  sensor_msgs::msg::LaserScan ros_msg;
  ros_msg.angle_min = 0.0f;
  ros_msg.angle_max = 1.0f;
  ros_msg.angle_increment = 0.1f;  // (1.0 - 0.0) / 0.1 == 10 by computation
  ros_msg.range_min = 0.0f;
  ros_msg.range_max = 10.0f;
  // ...but the publisher only filled 5 samples.
  ros_msg.ranges = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  ros_msg.intensities = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f};

  gz::msgs::LaserScan gz_msg;
  ASSERT_NO_FATAL_FAILURE(ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg));

  EXPECT_LE(static_cast<size_t>(gz_msg.ranges_size()), ros_msg.ranges.size());
  EXPECT_LE(
    static_cast<size_t>(gz_msg.intensities_size()),
    ros_msg.intensities.size());
}

TEST(LaserScanRosToGz, EmptyIntensitiesDoesNotOverread)
{
  sensor_msgs::msg::LaserScan ros_msg;
  ros_msg.angle_min = 0.0f;
  ros_msg.angle_max = 1.0f;
  ros_msg.angle_increment = 0.25f;
  ros_msg.ranges = {1.0f, 2.0f, 3.0f, 4.0f};
  // intensities deliberately empty — common for many real lidars.

  gz::msgs::LaserScan gz_msg;
  ASSERT_NO_FATAL_FAILURE(ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg));

  EXPECT_EQ(0, gz_msg.intensities_size());
}

// ---------------------------------------------------------------------------
// LaserScan GZ->ROS : trusts gz_msg.count() against the actual ranges
// array length. A malformed message with count > ranges_size() would
// cause an OOB read.
// ---------------------------------------------------------------------------
TEST(LaserScanGzToRos, CountLargerThanRangesArrayIsClamped)
{
  gz::msgs::LaserScan gz_msg;
  gz_msg.set_angle_min(0.0);
  gz_msg.set_angle_max(1.0);
  gz_msg.set_angle_step(0.1);
  gz_msg.set_range_min(0.0);
  gz_msg.set_range_max(10.0);
  // count says 10 but only 3 ranges are actually filled.
  gz_msg.set_count(10);
  gz_msg.set_vertical_count(1);
  for (int i = 0; i < 3; ++i) {
    gz_msg.add_ranges(static_cast<double>(i));
    gz_msg.add_intensities(static_cast<double>(i));
  }

  sensor_msgs::msg::LaserScan ros_msg;
  ASSERT_NO_FATAL_FAILURE(ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg));

  EXPECT_LE(ros_msg.ranges.size(), static_cast<size_t>(gz_msg.ranges_size()));
  EXPECT_LE(
    ros_msg.intensities.size(),
    static_cast<size_t>(gz_msg.intensities_size()));
}

// ---------------------------------------------------------------------------
// CameraInfo GZ->ROS : ROS arrays are fixed-size (k:9, p:12, r:9). A
// Gazebo publisher with more elements would write past the std::array.
// ---------------------------------------------------------------------------
TEST(CameraInfoGzToRos, IntrinsicsLongerThanRosArrayDoesNotOverflow)
{
  gz::msgs::CameraInfo gz_msg;
  gz_msg.set_width(640);
  gz_msg.set_height(480);
  auto * intrinsics = gz_msg.mutable_intrinsics();
  // Stuff 12 entries — ros_msg.k is std::array<double, 9>.
  for (int i = 0; i < 12; ++i) {
    intrinsics->add_k(static_cast<double>(i));
  }
  auto * projection = gz_msg.mutable_projection();
  // Stuff 16 entries — ros_msg.p is std::array<double, 12>.
  for (int i = 0; i < 16; ++i) {
    projection->add_p(static_cast<double>(i));
  }
  // Stuff 12 entries — ros_msg.r is std::array<double, 9>.
  for (int i = 0; i < 12; ++i) {
    gz_msg.add_rectification_matrix(static_cast<double>(i));
  }

  sensor_msgs::msg::CameraInfo ros_msg;
  ASSERT_NO_FATAL_FAILURE(ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg));

  // We only assert that the bridge truncated to the fixed-size arrays.
  // Anything else (or a crash) means the bug is still present.
  for (size_t i = 0; i < ros_msg.k.size(); ++i) {
    EXPECT_DOUBLE_EQ(static_cast<double>(i), ros_msg.k[i]);
  }
  for (size_t i = 0; i < ros_msg.p.size(); ++i) {
    EXPECT_DOUBLE_EQ(static_cast<double>(i), ros_msg.p[i]);
  }
  for (size_t i = 0; i < ros_msg.r.size(); ++i) {
    EXPECT_DOUBLE_EQ(static_cast<double>(i), ros_msg.r[i]);
  }
}
