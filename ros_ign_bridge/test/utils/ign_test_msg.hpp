// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef UTILS__IGN_TEST_MSG_HPP_
#define UTILS__IGN_TEST_MSG_HPP_

#include <ignition/msgs.hh>

#include <memory>

namespace ros_ign_bridge
{
namespace testing
{
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Boolean & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Boolean> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Color & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Color> & _msg);

/// \brief Compare a message with the populated for testing. Noop for Empty
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Empty> &);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Float & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Float> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Double & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Double> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::UInt32 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::UInt32> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Header & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Header> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Clock & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Clock> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::StringMsg & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::StringMsg> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Quaternion & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Quaternion> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Vector3d & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Vector3d> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Pose & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Pose> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Pose_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Pose_V> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Twist & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Twist> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Wrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Wrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::JointWrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::JointWrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Entity & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Entity> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Contact & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Contact> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Contacts & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Contacts> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Image & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Image> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::CameraInfo & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::CameraInfo> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::FluidPressure & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::FluidPressure> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::IMU & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::IMU> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Axis & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Axis> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Model & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Model> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::LaserScan & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::LaserScan> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Magnetometer & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Magnetometer> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Actuators & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Actuators> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Odometry & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Odometry> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::PointCloudPacked & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::PointCloudPacked> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::BatteryState & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::BatteryState> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::JointTrajectoryPoint & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::JointTrajectoryPoint> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::JointTrajectory & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::JointTrajectory> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ignition::msgs::Light & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ignition::msgs::Light> & _msg);

}  // namespace testing
}  // namespace ros_ign_bridge

#endif  // UTILS__IGN_TEST_MSG_HPP_
