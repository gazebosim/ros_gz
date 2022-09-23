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


#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>

#include <ros_ign_gazebo/Stopwatch.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <utility>


// NOLINTNEXTLINE
using namespace std::chrono;


class Stopwatch : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node{nullptr};
  rclcpp::SyncParametersClient::SharedPtr param_client{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor{nullptr};
  bool stop_spinning_{false};
  std::thread spin_thread;

  virtual void SetUp()
  {
    if (spin_thread.joinable()) {
      stop_spinning_ = true;
      spin_thread.join();
    }
    stop_spinning_ = false;
    // Shutdown in case there was a dangling global context from other test fixtures
    rclcpp::shutdown();
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("clock_sleep_node");
    param_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    ASSERT_TRUE(param_client->wait_for_service(5s));
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }

  void TearDown()
  {
    if (spin_thread.joinable()) {
      stop_spinning_ = true;
      spin_thread.join();
    }
    executor.reset();
    node.reset();
    rclcpp::shutdown();
  }
};

/////////////////////////////////////////////////
// Helper function that runs a few tests
void runTimer(ros_ign_gazebo::Stopwatch & _time)
{
  // Start the timer
  EXPECT_TRUE(_time.Start());
  // The timer should be running.
  EXPECT_TRUE(_time.Running());
  // The start time should be greater than the stop time.
  EXPECT_GT(_time.StartTime(), _time.StopTime());
  // The elapsed stop time should still be zero.
  EXPECT_EQ(
    rclcpp::Duration(0, 0U),
    _time.ElapsedStopTime());

  // Wait for some time...
  std::this_thread::sleep_for(1000ms);
  // Now the elapsed time should be greater than or equal to the time slept.
  EXPECT_GE(
    _time.ElapsedRunTime(),
    rclcpp::Duration(1, 0U));

  // Stop the timer.
  EXPECT_TRUE(_time.Stop());
  // The timer should not be running.
  EXPECT_FALSE(_time.Running());
  // The stop time should be greater than the start time.
  EXPECT_GT(_time.StopTime(), _time.StartTime());
  // The elapsed time should still be greater than the time slept.
  EXPECT_GE(
    _time.ElapsedRunTime(),
    rclcpp::Duration(1, 0U));

  // Save the elapsed time.
  auto elapsedTime = _time.ElapsedRunTime();

  // The timer is now stopped, let's sleep some more.
  std::this_thread::sleep_for(1000ms);
  // The elapsed stop time should be greater than or equal to the time
  // slept.
  EXPECT_GE(
    _time.ElapsedStopTime(),
    rclcpp::Duration(1, 0U));
  // The elapsed time should be the same.
  EXPECT_EQ(elapsedTime, _time.ElapsedRunTime());

  // Start the timer again.
  EXPECT_TRUE(_time.Start());
  // Store the elapsed stop time.
  auto elapsedStopTime = _time.ElapsedStopTime();
  // The timer should be running.
  EXPECT_TRUE(_time.Running());
  // Sleep for some time.
  std::this_thread::sleep_for(1000ms);
  // The elapsed stop time should remain the same
  EXPECT_EQ(elapsedStopTime, _time.ElapsedStopTime());
  // The elapsed time should be greater than the previous elapsed time.
  EXPECT_GT(_time.ElapsedRunTime(), elapsedTime);
  // The elapsed time should be greater than or equal to the the previous
  // two sleep times.
  EXPECT_GE(
    _time.ElapsedRunTime(),
    rclcpp::Duration(2, 0U));
}

/////////////////////////////////////////////////
TEST_F(Stopwatch, SimTime)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", true)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  executor->add_node(node);

  ros_ign_gazebo::Stopwatch watch;
  watch.SetClock(clock);

  EXPECT_FALSE(watch.Running());
  EXPECT_EQ(watch.StopTime(), watch.StartTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedRunTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedStopTime());

  // Start the timer
  EXPECT_TRUE(watch.Start());
  // The timer should be running.
  EXPECT_TRUE(watch.Running());
  // The start time should still be equal to the stop time
  // since both started at 0
  EXPECT_EQ(watch.StartTime(), watch.StopTime());
  // The elapsed run time should still be zero.
  EXPECT_EQ(
    rclcpp::Duration(0, 0U),
    watch.ElapsedRunTime());
  // The elapsed stop time should still be zero.
  EXPECT_EQ(
    rclcpp::Duration(0, 0U),
    watch.ElapsedStopTime());

  // move `/clock` forward for a certain amount of time
  auto new_time = clock->now() + rclcpp::Duration(1, 0U);
  auto msg = rosgraph_msgs::msg::Clock();
  msg.clock = rclcpp::Time(new_time);
  clock_publisher->publish(msg);
  while (rclcpp::ok() && clock->now() < new_time) {
    executor->spin_once(10ms);
  }

  // Now the elapsed time should be greater than or equal to the time slept.
  EXPECT_GE(
    watch.ElapsedRunTime(),
    rclcpp::Duration(1, 0U));

  // Stop the timer.
  EXPECT_TRUE(watch.Stop());
  // The timer should not be running.
  EXPECT_FALSE(watch.Running());
  // The stop time should be greater than the start time.
  EXPECT_GT(watch.StopTime(), watch.StartTime());
  // The elapsed time should still be greater than the time slept.
  EXPECT_GE(
    watch.ElapsedRunTime(),
    rclcpp::Duration(1, 0U));

  // Save the elapsed time.
  auto elapsedTime = watch.ElapsedRunTime();

  // The timer is now stopped, let's
  // move `/clock` forward for a certain amount of time
  new_time = clock->now() + rclcpp::Duration(1, 0U);
  msg = rosgraph_msgs::msg::Clock();
  msg.clock = rclcpp::Time(new_time);
  clock_publisher->publish(msg);
  while (rclcpp::ok() && clock->now() < new_time) {
    executor->spin_once(10ms);
  }
  // The elapsed stop time should be greater than or equal to the time
  // moved.
  EXPECT_GE(
    watch.ElapsedStopTime(),
    rclcpp::Duration(1, 0U));
  // The elapsed run time should be the same.
  EXPECT_EQ(elapsedTime, watch.ElapsedRunTime());

  // Start the timer again.
  EXPECT_TRUE(watch.Start());
  // Store the elapsed stop time.
  auto elapsedStopTime = watch.ElapsedStopTime();
  // The timer should be running.
  EXPECT_TRUE(watch.Running());
  // Now start time should be greater than stop
  EXPECT_GT(watch.StartTime(), watch.StopTime());
  // move `/clock` forward for a certain amount of time
  new_time = clock->now() + rclcpp::Duration(1, 0U);
  msg = rosgraph_msgs::msg::Clock();
  msg.clock = rclcpp::Time(new_time);
  clock_publisher->publish(msg);
  while (rclcpp::ok() && clock->now() < new_time) {
    executor->spin_once(10ms);
  }
  // The elapsed stop time should remain the same
  EXPECT_EQ(elapsedStopTime, watch.ElapsedStopTime());
  // The elapsed time should be greater than the previous elapsed time.
  EXPECT_GT(watch.ElapsedRunTime(), elapsedTime);
  // The elapsed time should be greater than or equal to the the previous
  // two sleep times.
  EXPECT_GE(
    watch.ElapsedRunTime(),
    rclcpp::Duration(2, 0U));
}

/////////////////////////////////////////////////
TEST_F(Stopwatch, Constructor)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", false)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  executor->add_node(node);
  auto spin = [this]()
    {
      while (rclcpp::ok() && !this->stop_spinning_) {
        this->executor->spin_once(10ms);
      }
    };
  spin_thread = std::thread(spin);

  ros_ign_gazebo::Stopwatch watch;
  watch.SetClock(clock);

  EXPECT_FALSE(watch.Running());
  EXPECT_EQ(watch.StopTime(), watch.StartTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedRunTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedStopTime());

  runTimer(watch);

  ros_ign_gazebo::Stopwatch watch2(watch);
  EXPECT_EQ(watch, watch2);

  ros_ign_gazebo::Stopwatch watch3(std::move(watch2));
  EXPECT_EQ(watch, watch3);
}

/////////////////////////////////////////////////
TEST_F(Stopwatch, EqualOperator)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", false)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  executor->add_node(node);
  auto spin = [this]()
    {
      while (rclcpp::ok() && !this->stop_spinning_) {
        this->executor->spin_once(10ms);
      }
    };
  spin_thread = std::thread(spin);

  ros_ign_gazebo::Stopwatch watch;
  watch.SetClock(clock);
  ros_ign_gazebo::Stopwatch watch2;
  watch2.SetClock(clock);
  ros_ign_gazebo::Stopwatch watch3;
  watch3.SetClock(clock);

  EXPECT_EQ(watch, watch2);
  EXPECT_EQ(watch, watch3);

  runTimer(watch);
  runTimer(watch2);
  runTimer(watch3);

  EXPECT_NE(watch, watch2);
  EXPECT_NE(watch, watch3);

  watch2 = watch;
  EXPECT_EQ(watch, watch2);

  watch3 = std::move(watch2);
  EXPECT_EQ(watch, watch3);
}

/////////////////////////////////////////////////
TEST_F(Stopwatch, StartStopReset)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", false)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  executor->add_node(node);
  auto spin = [this]()
    {
      while (rclcpp::ok() && !this->stop_spinning_) {
        this->executor->spin_once(10ms);
      }
    };
  spin_thread = std::thread(spin);

  ros_ign_gazebo::Stopwatch watch;
  watch.SetClock(clock);
  runTimer(watch);

  watch.Reset();

  EXPECT_FALSE(watch.Running());
  EXPECT_EQ(watch.StopTime(), watch.StartTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedRunTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedStopTime());

  runTimer(watch);

  EXPECT_TRUE(watch.Running());

  watch.Start(true);
  EXPECT_TRUE(watch.Running());
  EXPECT_LT(watch.StopTime(), watch.StartTime());
  EXPECT_NE(rclcpp::Duration(0, 0U), watch.ElapsedRunTime());
  EXPECT_EQ(rclcpp::Duration(0, 0U), watch.ElapsedStopTime());
}

/////////////////////////////////////////////////
TEST_F(Stopwatch, FailStartStop)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", false)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  executor->add_node(node);
  auto spin = [this]()
    {
      while (rclcpp::ok() && !this->stop_spinning_) {
        this->executor->spin_once(10ms);
      }
    };
  spin_thread = std::thread(spin);

  ros_ign_gazebo::Stopwatch watch;
  watch.SetClock(clock);

  // Can't stop while not running
  EXPECT_FALSE(watch.Stop());
  EXPECT_FALSE(watch.Running());

  // Can start while not running
  EXPECT_TRUE(watch.Start());
  EXPECT_TRUE(watch.Running());

  // Can't start while running
  EXPECT_FALSE(watch.Start());
  EXPECT_TRUE(watch.Running());

  // Can stop while running
  EXPECT_TRUE(watch.Stop());
  EXPECT_FALSE(watch.Running());

  // Can start while not running
  EXPECT_TRUE(watch.Start());
  EXPECT_TRUE(watch.Running());
}
