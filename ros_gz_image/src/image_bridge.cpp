// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <rmw/qos_profiles.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>

//////////////////////////////////////////////////
/// \brief Bridges one image topic
class Handler
{
public:
  /// \brief Constructor
  /// \param[in] _topic Image base topic
  /// \param[in] _node Pointer to ROS node
  /// \param[in] _gz_node Pointer to Gazebo node
  /// \param[in] _lazy Whether to lazily subscribe to Gazebo only when ROS subscribers are present
  Handler(
    const std::string & _topic,
    std::shared_ptr<rclcpp::Node> _node,
    std::shared_ptr<gz::transport::Node> _gz_node,
    bool _lazy = false)
  : topic_(_topic),
    gz_node_(_gz_node),
    is_lazy_(_lazy)
  {
    // Get QoS profile from parameter
    rclcpp::QoS qos_profile = rclcpp::QoS(10);
    const auto qos_str =
      _node->get_parameter("qos").get_parameter_value().get<std::string>();
    if (qos_str == "system_default") {
      qos_profile = rclcpp::SystemDefaultsQoS();
    } else if (qos_str == "sensor_data") {
      qos_profile = rclcpp::SensorDataQoS();
    } else if (qos_str != "default") {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Invalid QoS profile %s specified. Using default profile.",
        qos_str.c_str());
    }

    // Always create the ROS publisher
    this->ros_pub_ = image_transport::create_publisher(
      *_node, _topic, qos_profile);

    // Subscribe to Gazebo immediately unless lazy
    if (!is_lazy_) {
      StartSubscriber();
    }
  }

  /// \brief Manage Gazebo subscription lifecycle based on ROS subscriber count.
  /// Only active when lazy mode is enabled.
  void CheckSubscribers()
  {
    if (!is_lazy_) {
      return;
    }

    const size_t num_subs = ros_pub_.getNumSubscribers();

    if (has_subscriber_ && num_subs == 0) {
      StopSubscriber();
    } else if (!has_subscriber_ && num_subs > 0) {
      StartSubscriber();
    }
  }

private:
  void StartSubscriber()
  {
    gz_node_->Subscribe(topic_, &Handler::OnImage, this);
    has_subscriber_ = true;
  }

  void StopSubscriber()
  {
    gz_node_->Unsubscribe(topic_);
    has_subscriber_ = false;
  }

  /// \brief Callback when Gazebo image is received
  /// \param[in] _gz_msg Gazebo message
  void OnImage(const gz::msgs::Image & _gz_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
    this->ros_pub_.publish(ros_msg);
  }

  /// \brief Image topic name
  std::string topic_;

  /// \brief Gazebo transport node
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief Whether to use lazy subscription
  bool is_lazy_{false};

  /// \brief Whether currently subscribed to Gazebo topic
  std::atomic<bool> has_subscriber_{false};

  /// \brief ROS image publisher
  image_transport::Publisher ros_pub_;
};

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Bridge a collection of Gazebo Transport image topics to ROS " <<
    "using image_transport.\n\n" <<
    "  image_bridge <topic> <topic> ..\n\n" <<
    "Optional ROS parameters:\n" <<
    "  qos:=<profile>              QoS profile: default, sensor_data, system_default\n" <<
    "  lazy:=<true|false>          Only subscribe to Gazebo when ROS subscribers are present\n" <<
    "  subscription_heartbeat:=<ms> Interval (ms) to check ROS subscriber count (default: 1000)\n\n"
            <<
    "E.g.: image_bridge /camera/front/image_raw\n" <<
    "E.g.: image_bridge /camera/front/image_raw --ros-args -p lazy:=true" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }

  rclcpp::init(argc, argv);

  // ROS node
  auto node_ = rclcpp::Node::make_shared("ros_gz_image");
  node_->declare_parameter("qos", "default");
  node_->declare_parameter("lazy", false);
  node_->declare_parameter("subscription_heartbeat", 1000);

  const bool lazy = node_->get_parameter("lazy").as_bool();
  const int heartbeat_ms = node_->get_parameter("subscription_heartbeat").as_int();

  if (lazy && heartbeat_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "subscription_heartbeat must be a positive integer (got %d). Using default of 1000 ms.",
      heartbeat_ms);
    rclcpp::shutdown();
    return -1;
  }

  // Gazebo node
  auto gz_node = std::make_shared<gz::transport::Node>();

  std::vector<std::shared_ptr<Handler>> handlers;

  // skip the process name in argument processing
  ++argv;
  --argc;
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  // Create publishers (and subscribers if not lazy)
  for (const auto & topic : args) {
    handlers.push_back(std::make_shared<Handler>(topic, node_, gz_node, lazy));
  }

  // When lazy, periodically check ROS subscriber count to start/stop Gz subscriptions
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  if (lazy) {
    heartbeat_timer = node_->create_wall_timer(
      std::chrono::milliseconds(heartbeat_ms),
      [&handlers]() {
        for (auto & handler : handlers) {
          handler->CheckSubscribers();
        }
      });
  }

  // Spin ROS and Gz until shutdown
  rclcpp::spin(node_);

  return 0;
}
