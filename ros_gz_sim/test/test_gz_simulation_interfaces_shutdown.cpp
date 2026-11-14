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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "ros_gz_sim/gz_simulation_interfaces.hpp"

using namespace std::chrono_literals;

class GzSimulationInterfacesShutdownTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setenv("GZ_IP", "127.0.0.1", 1);
    setenv("GZ_PARTITION", "test_gz_sim_interfaces_no_gazebo", 1);
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(GzSimulationInterfacesShutdownTest, DestroysPromptlyAndReleasesSubNodeWithoutGazebo)
{
  auto parent_node = std::make_shared<rclcpp::Node>("test_gzserver");
  const auto baseline_node_base_use_count =
    parent_node->get_node_base_interface().use_count();

  auto sub_node = parent_node->create_sub_node(parent_node->get_name());
  std::weak_ptr<rclcpp::Node> weak_sub_node = sub_node;

  auto sim_interfaces =
    std::make_unique<ros_gz_sim::gz_simulation_interfaces::GzSimulationInterfaces>(
      std::move(sub_node));

  // Give the background initialization thread a moment to enter its wait for Gazebo.
  std::this_thread::sleep_for(50ms);

  const auto start = std::chrono::steady_clock::now();
  sim_interfaces.reset();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 1s);
  EXPECT_TRUE(weak_sub_node.expired());
  EXPECT_EQ(
    parent_node->get_node_base_interface().use_count(),
    baseline_node_base_use_count);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
