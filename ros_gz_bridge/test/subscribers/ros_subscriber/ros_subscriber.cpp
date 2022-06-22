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


#include <gtest/gtest.h>

#include <memory>

#include "ros_subscriber.hpp"

static std::shared_ptr<rclcpp::Node> kTestNode;

/////////////////////////////////////////////////
rclcpp::Node * ros_subscriber::TestNode()
{
  if (kTestNode == nullptr) {
    kTestNode = rclcpp::Node::make_shared("ros_subscriber");
  }
  return kTestNode.get();
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  kTestNode.reset();
  return ret;
}
