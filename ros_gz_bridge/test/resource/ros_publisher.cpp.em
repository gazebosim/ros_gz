// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include "utils/test_utils.hpp"
#include "utils/ros_test_msg.hpp"

//////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_string_publisher");
  rclcpp::Rate loop_rate(100);

@[for m in mappings]@
  // @(m.ros2_string()).
  auto @(m.unique())_pub =
    node->create_publisher<@(m.ros2_type())>("@(m.unique())", 1);
  @(m.ros2_type()) @(m.unique())_msg;
  ros_gz_bridge::testing::createTestMsg(@(m.unique())_msg);

@[end for]@

  while (rclcpp::ok()) {
    // Publish all messages.
@[for m in mappings]@
    @(m.unique())_pub->publish(@(m.unique())_msg);
@[end for]@

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
