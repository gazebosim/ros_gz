// Copyright 2026 Open Source Robotics Foundation
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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <gz/transport.hh>
#include <rclcpp/rclcpp.hpp>

#include <ros_gz_bridge/ros_gz_bridge.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>
#include <std_msgs/msg/string.hpp>

#include "bridge_handle.hpp"
#include "utils/gz_test_msg.hpp"
#include "utils/ros_test_msg.hpp"

using namespace std::chrono_literals;

class TestableRosGzBridge : public ros_gz_bridge::RosGzBridge
{
public:
  explicit TestableRosGzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : RosGzBridge(options)
  {
  }

  using RosGzBridge::create_automated_bridges;

  size_t topic_bridge_count() const
  {
    return this->handles_.size();
  }

  size_t service_bridge_count() const
  {
    return this->services_.size();
  }

  size_t topic_bridge_count(const std::string & topic_name) const
  {
    size_t count = 0;
    for (const auto & handle : this->handles_) {
      if (handle->GetConfig().gz_topic_name == topic_name) {
        ++count;
      }
    }
    return count;
  }

  size_t service_bridge_count(const std::string & service_name) const
  {
    size_t count = 0;
    for (const auto & service : this->services_) {
      if (service->get_service_name() == service_name) {
        ++count;
      }
    }
    return count;
  }

  bool check_gz_topic(const std::string & topic_name) const
  {
    std::vector<std::string> topics;
    this->gz_node_->TopicList(topics);

    return std::find(topics.begin(), topics.end(), topic_name) != topics.end();
  }

  bool check_gz_service(const std::string & service_name) const
  {
    std::vector<std::string> services;
    this->gz_node_->ServiceList(services);

    return std::find(services.begin(), services.end(), service_name) != services.end();
  }
};

template<typename T>
class RosPublisher
{
public:
  RosPublisher(const rclcpp::Node::SharedPtr & node, const std::string & topic_name)
  {
    this->pub_ = node->create_publisher<T>(topic_name, 10);
  }

  void Publish(const T & msg) const
  {
    this->pub_->publish(msg);
  }

private:
  typename rclcpp::Publisher<T>::SharedPtr pub_;
};

template<typename T>
class GzPublisher
{
public:
  GzPublisher(gz::transport::Node & node, const std::string & topic_name)
  {
    this->pub_ = node.Advertise<T>(topic_name);
  }

  void Publish(const T & msg)
  {
    this->pub_.Publish(msg);
  }

private:
  gz::transport::Node::Publisher pub_;
};

template<typename T>
class RosSubscriber
{
public:
  RosSubscriber(const rclcpp::Node::SharedPtr & node, const std::string & topic_name)
  {
    this->sub_ = node->create_subscription<T>(
      topic_name,
      10,
      [this](const T & msg)
      {
        this->msg_ = msg;
        this->received_ = true;
      });
  }

  bool received() const
  {
    return this->received_;
  }

  T message() const
  {
    return this->msg_;
  }

private:
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  bool received_{false};
  T msg_;
};

template<typename T>
class GzSubscriber
{
public:
  GzSubscriber(gz::transport::Node & node, const std::string & topic_name)
  {
    this->subscribed_ = node.Subscribe(
      topic_name, &GzSubscriber::OnMessage, this);
  }

  bool subscribed() const
  {
    return this->subscribed_;
  }

  bool received() const
  {
    return this->received_;
  }

  T message() const
  {
    return this->msg_;
  }

private:
  void OnMessage(const T & msg)
  {
    this->msg_ = msg;
    this->received_ = true;
  }

  bool subscribed_{false};
  bool received_{false};
  T msg_;
};

template<typename ReqT, typename RepT>
class GzServer
{
public:
  GzServer(gz::transport::Node & node, const std::string & service_name)
  {
    this->advertised_ = node.Advertise(
      service_name,
      &GzServer::HandleRequest,
      this);
  }

  bool advertised() const
  {
    return this->advertised_;
  }

  bool called() const
  {
    return this->called_;
  }

private:
  bool HandleRequest(const ReqT &, RepT & response)
  {
    response.set_data(true);
    this->called_ = true;
    return true;
  }

  bool advertised_{false};
  bool called_{false};
};

template<typename T>
class RosClient
{
public:
  RosClient(const rclcpp::Node::SharedPtr & node, const std::string & service_name)
  {
    this->client_ = node->create_client<T>(service_name);
  }

  bool available() const
  {
    return this->client_->wait_for_service(0s);
  }

  auto SendRequest()
  {
    auto request =
      std::make_shared<typename T::Request>();

    return this->client_->async_send_request(request);
  }

private:
  typename rclcpp::Client<T>::SharedPtr client_;
};

class AutomatedBridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    this->bridge_ = std::make_shared<TestableRosGzBridge>();

    this->ros_node_ = std::make_shared<rclcpp::Node>("automated_bridge_test");
  }

  void TearDown() override
  {
    this->ros_node_.reset();
    this->bridge_.reset();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<TestableRosGzBridge> bridge_;
  rclcpp::Node::SharedPtr ros_node_;
  gz::transport::Node gz_node_;
};

TEST_F(AutomatedBridgeTest, GzToRosWhenRosSubExist)
{
  const std::string topic_name = "/auto_gz_to_ros";
  GzPublisher<gz::msgs::StringMsg> gz_pub(this->gz_node_, topic_name);
  RosSubscriber<std_msgs::msg::String> ros_sub(this->ros_node_, topic_name);

  rclcpp::WallRate rate(20.0);

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->topic_bridge_count(topic_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->topic_bridge_count(topic_name));

  gz::msgs::StringMsg msg;
  ros_gz_bridge::testing::createTestMsg(msg);

  for (int i = 0; i < 50 && !ros_sub.received(); ++i) {
    gz_pub.Publish(msg);

    rclcpp::spin_some(this->bridge_);
    rclcpp::spin_some(this->ros_node_);

    rate.sleep();
  }

  ASSERT_TRUE(ros_sub.received());

  ros_gz_bridge::testing::compareTestMsg(
    std::make_shared<std_msgs::msg::String>(ros_sub.message()));
}

TEST_F(AutomatedBridgeTest, GzToRosSkipUntilRosSubAppear)
{
  const std::string topic_name = "/auto_gz_to_ros_late_sub";
  GzPublisher<gz::msgs::StringMsg> gz_pub(this->gz_node_, topic_name);

  rclcpp::WallRate rate(20.0);

  // Make sure Gazebo discovery has seen the publisher.
  for (int i = 0; i < 50; ++i) {
    if (this->bridge_->check_gz_topic(topic_name)) {
      break;
    }

    rate.sleep();
  }

  ASSERT_TRUE(this->bridge_->check_gz_topic(topic_name));

  for (int i = 0; i < 5; ++i) {
    this->bridge_->create_automated_bridges();
    rate.sleep();
  }
  // There is no ROS subscriber yet, so no bridge should be created.
  EXPECT_EQ(0, this->bridge_->topic_bridge_count(topic_name));

  RosSubscriber<std_msgs::msg::String> ros_sub(this->ros_node_, topic_name);

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->topic_bridge_count(topic_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->topic_bridge_count(topic_name));

  gz::msgs::StringMsg msg;
  ros_gz_bridge::testing::createTestMsg(msg);

  for (int i = 0; i < 50 && !ros_sub.received(); ++i) {
    gz_pub.Publish(msg);

    rclcpp::spin_some(this->bridge_);
    rclcpp::spin_some(this->ros_node_);

    rate.sleep();
  }

  ASSERT_TRUE(ros_sub.received());

  ros_gz_bridge::testing::compareTestMsg(
    std::make_shared<std_msgs::msg::String>(ros_sub.message()));
}

TEST_F(AutomatedBridgeTest, RosToGzWhenGzSubExist)
{
  const std::string topic_name = "/auto_ros_to_gz";
  RosPublisher<std_msgs::msg::String> ros_pub(this->ros_node_, topic_name);
  GzSubscriber<gz::msgs::StringMsg> gz_sub(this->gz_node_, topic_name);

  ASSERT_TRUE(gz_sub.subscribed());

  rclcpp::WallRate rate(20.0);

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->topic_bridge_count(topic_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->topic_bridge_count(topic_name));

  std_msgs::msg::String msg;
  ros_gz_bridge::testing::createTestMsg(msg);

  for (int i = 0; i < 50 && !gz_sub.received(); ++i) {
    ros_pub.Publish(msg);
    rclcpp::spin_some(this->bridge_);

    rate.sleep();
  }

  ASSERT_TRUE(gz_sub.received());

  ros_gz_bridge::testing::compareTestMsg(
    std::make_shared<gz::msgs::StringMsg>(gz_sub.message()));
}

TEST_F(AutomatedBridgeTest, RosToGzSkipUntilGzSubAppear)
{
  const std::string topic_name = "/auto_ros_to_gz_late_sub";
  RosPublisher<std_msgs::msg::String> ros_pub(this->ros_node_, topic_name);

  rclcpp::WallRate rate(20.0);

  for (int i = 0; i < 5; ++i) {
    this->bridge_->create_automated_bridges();
    rate.sleep();
  }
  // There is no Gazebo subscriber yet, so no bridge should be created.
  EXPECT_EQ(0, this->bridge_->topic_bridge_count(topic_name));

  GzSubscriber<gz::msgs::StringMsg> gz_sub(this->gz_node_, topic_name);
  ASSERT_TRUE(gz_sub.subscribed());

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->topic_bridge_count(topic_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->topic_bridge_count(topic_name));

  std_msgs::msg::String msg;
  ros_gz_bridge::testing::createTestMsg(msg);

  for (int i = 0; i < 50 && !gz_sub.received(); ++i) {
    ros_pub.Publish(msg);

    rclcpp::spin_some(this->bridge_);

    rate.sleep();
  }

  ASSERT_TRUE(gz_sub.received());

  ros_gz_bridge::testing::compareTestMsg(
    std::make_shared<gz::msgs::StringMsg>(gz_sub.message()));
}

TEST_F(AutomatedBridgeTest, ServiceWhenRosClientExist)
{
  const std::string service_name = "/auto_control_world";

  GzServer<gz::msgs::WorldControl, gz::msgs::Boolean>
    gz_server(this->gz_node_, service_name);

  RosClient<ros_gz_interfaces::srv::ControlWorld>
    ros_client(this->ros_node_, service_name);

  ASSERT_TRUE(gz_server.advertised());

  rclcpp::WallRate rate(20.0);

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->service_bridge_count(service_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->service_bridge_count(service_name));

  for (int i = 0; i < 50 && !ros_client.available(); ++i) {
    rate.sleep();
  }

  ASSERT_TRUE(ros_client.available());

  auto future = ros_client.SendRequest();

  for (int i = 0; i < 100; ++i) {
    rclcpp::spin_some(this->bridge_);
    rclcpp::spin_some(this->ros_node_);

    if (future.wait_for(0s) == std::future_status::ready) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(std::future_status::ready, future.wait_for(0s));

  const auto response = future.get();

  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(gz_server.called());
}

TEST_F(AutomatedBridgeTest, ServiceSkipUntilRosClientAppear)
{
  const std::string service_name = "/auto_control_world_late_client";

  GzServer<gz::msgs::WorldControl, gz::msgs::Boolean>
    gz_server(this->gz_node_, service_name);

  ASSERT_TRUE(gz_server.advertised());

  rclcpp::WallRate rate(20.0);

  // Make sure the Gazebo service is visible.
  for (int i = 0; i < 50; ++i) {
    if (this->bridge_->check_gz_service(service_name)) {
      break;
    }

    rate.sleep();
  }

  ASSERT_TRUE(this->bridge_->check_gz_service(service_name));

  for (int i = 0; i < 5; ++i) {
    this->bridge_->create_automated_bridges();
    rate.sleep();
  }

  // There is no ROS client yet, so no bridge should be created.
  EXPECT_EQ(0, this->bridge_->service_bridge_count(service_name));

  RosClient<ros_gz_interfaces::srv::ControlWorld>
    ros_client(this->ros_node_, service_name);

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->service_bridge_count(service_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->service_bridge_count(service_name));

  for (int i = 0; i < 50 && !ros_client.available(); ++i) {
    rate.sleep();
  }

  ASSERT_TRUE(ros_client.available());

  auto future = ros_client.SendRequest();

  for (int i = 0; i < 100; ++i) {
    rclcpp::spin_some(this->bridge_);
    rclcpp::spin_some(this->ros_node_);

    if (future.wait_for(0s) == std::future_status::ready) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(std::future_status::ready, future.wait_for(0s));

  const auto response = future.get();

  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(gz_server.called());
}

TEST_F(AutomatedBridgeTest, AvoidDuplicateCreation)
{
  const std::string topic_name = "/auto_no_duplicate_topic";

  GzPublisher<gz::msgs::StringMsg> gz_pub(this->gz_node_, topic_name);

  RosSubscriber<std_msgs::msg::String> ros_sub(this->ros_node_, topic_name);

  rclcpp::WallRate rate(20.0);

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->topic_bridge_count(topic_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->topic_bridge_count(topic_name));

  for (int i = 0; i < 3; ++i) {
    this->bridge_->create_automated_bridges();
    rate.sleep();
  }

  EXPECT_EQ(1, this->bridge_->topic_bridge_count(topic_name));

  const std::string service_name = "/auto_no_duplicate_service";

  GzServer<gz::msgs::WorldControl, gz::msgs::Boolean>
    gz_server(this->gz_node_, service_name);

  RosClient<ros_gz_interfaces::srv::ControlWorld>
    ros_client(this->ros_node_, service_name);

  ASSERT_TRUE(gz_server.advertised());

  for (int i = 0; i < 50; ++i) {
    this->bridge_->create_automated_bridges();

    if (this->bridge_->service_bridge_count(service_name) == 1) {
      break;
    }

    rate.sleep();
  }

  ASSERT_EQ(1, this->bridge_->service_bridge_count(service_name));

  for (int i = 0; i < 3; ++i) {
    this->bridge_->create_automated_bridges();
    rate.sleep();
  }

  EXPECT_EQ(1, this->bridge_->service_bridge_count(service_name));
}
