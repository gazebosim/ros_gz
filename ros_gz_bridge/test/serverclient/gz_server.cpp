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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <gz/transport.hh>

#include "utils/test_utils.hpp"
#include "utils/gz_test_msg.hpp"

/// \brief Flag used to break the waiting loop and terminate the program.
static std::atomic_flag g_terminateSrv(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that handles requests
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM) {
    g_terminateSrv.clear();
  }
}

//////////////////////////////////////////////////
bool control_world(
  const gz::msgs::WorldControl &,
  gz::msgs::Boolean & _res)
{
  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
int main(int /*argc*/, char **/*argv*/)
{
  g_terminateSrv.test_and_set();
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  gz::transport::Node node;

  // gz::msgs::WorldControl.
  node.Advertise(
    "/gz_ros/test/serviceclient/world_control",
    &control_world);

  // Requests are handled in another thread.
  // Wait until a signal is sent.
  while (g_terminateSrv.test_and_set()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
