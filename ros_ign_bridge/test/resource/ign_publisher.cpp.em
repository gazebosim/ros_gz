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

// This file is generated from test/resource/ign_publisher.cpp.em

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "utils/test_utils.hpp"
#include "utils/ign_test_msg.hpp"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
/// \param[in] _signal signal number (interrupt or terminate)
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM) {
    g_terminatePub = true;
  }
}

//////////////////////////////////////////////////
int main(int /*argc*/, char **/*argv*/)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  ignition::transport::Node node;

@[for m in mappings]@
  // @(m.ign_string()).
  auto @(m.unique())_pub =
    node.Advertise<@(m.ign_type())>("@(m.unique())");
  @(m.ign_type()) @(m.unique())_msg;
  ros_ign_bridge::testing::createTestMsg(@(m.unique())_msg);

@[end for]@

  // Publish messages at 100 Hz.
  while (!g_terminatePub) {
@[for m in mappings]@
    @(m.unique())_pub.Publish(@(m.unique())_msg);
@[end for]@
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
