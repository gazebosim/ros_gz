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

// Regression test for gazebosim/ros_gz#979: image_bridge must exit cleanly on
// SIGINT while Gazebo image frames are still arriving. Gazebo transport does
// not wait for in-flight callbacks when a subscription or node is destroyed,
// so a frame delivered during teardown used to reach a destroyed Handler.

#include <gtest/gtest.h>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

extern char ** environ;

namespace
{
constexpr int kTrials = 20;
constexpr auto kRunTime = std::chrono::milliseconds(1000);
constexpr auto kPublishPeriod = std::chrono::microseconds(3333);  // ~300 Hz

/// \brief Publishes 640x480 RGB frames on two topics until stopped.
class FramePublisher
{
public:
  FramePublisher()
  {
    auto pub_a = node_.Advertise<gz::msgs::Image>("/teardown_cam_a");
    auto pub_b = node_.Advertise<gz::msgs::Image>("/teardown_cam_b");
    thread_ = std::thread([this, pub_a, pub_b]() mutable {
          gz::msgs::Image img;
          img.set_width(640);
          img.set_height(480);
          img.set_step(640 * 3);
          img.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
          img.set_data(std::string(640 * 480 * 3, '\x7f'));
          while (running_) {
            pub_a.Publish(img);
            pub_b.Publish(img);
            std::this_thread::sleep_for(kPublishPeriod);
          }
        });
  }

  ~FramePublisher()
  {
    running_ = false;
    thread_.join();
  }

private:
  gz::transport::Node node_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};
}  // namespace

TEST(ImageBridgeTeardown, ExitsCleanlyOnSigintWhileFramesArrive)
{
  // Isolate this test's Gazebo traffic. The bridge inherits the environment.
  const std::string partition = "image_bridge_teardown_" + std::to_string(getpid());
  ASSERT_EQ(0, setenv("GZ_PARTITION", partition.c_str(), 1));

  FramePublisher publisher;

  for (int trial = 0; trial < kTrials; ++trial) {
    char arg0[] = IMAGE_BRIDGE_EXECUTABLE;
    char arg1[] = "/teardown_cam_a";
    char arg2[] = "/teardown_cam_b";
    char * argv[] = {arg0, arg1, arg2, nullptr};

    pid_t pid;
    ASSERT_EQ(0, posix_spawn(&pid, IMAGE_BRIDGE_EXECUTABLE, nullptr, nullptr, argv, environ));

    std::this_thread::sleep_for(kRunTime);
    ASSERT_EQ(0, kill(pid, SIGINT));

    int status = 0;
    ASSERT_EQ(pid, waitpid(pid, &status, 0));
    ASSERT_FALSE(WIFSIGNALED(status)) <<
      "image_bridge was killed by signal " << WTERMSIG(status) << " in trial " << trial;
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(0, WEXITSTATUS(status)) << "in trial " << trial;
  }
}
