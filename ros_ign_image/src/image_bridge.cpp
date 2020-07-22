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

#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ros_ign_bridge/convert.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

//////////////////////////////////////////////////
/// \brief Bridges one topic
class Handler
{
public:
  /// \brief Constructor
  /// \param[in] _topic Image base topic
  /// \param[in] _it_node Pointer to image transport node
  /// \param[in] _ign_node Pointer to Ignition node
  Handler(
    const std::string & _topic,
    std::shared_ptr<image_transport::ImageTransport> _it_node,
    std::shared_ptr<ignition::transport::Node> _ign_node)
  {
    this->ros_pub = _it_node->advertise(_topic, 1);

    _ign_node->Subscribe(_topic, &Handler::OnImage, this);
  }

private:
  /// \brief Callback when Ignition image is received
  /// \param[in] _ign_msg Ignition message
  void OnImage(const ignition::msgs::Image & _ign_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_ign_bridge::convert_ign_to_ros(_ign_msg, ros_msg);
    this->ros_pub.publish(ros_msg);
  }

  /// \brief ROS image publisher
  image_transport::Publisher ros_pub;
};

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Bridge a collection of Ignition Transport image topics to ROS " <<
    "using image_transport.\n\n" <<
    "  image_bridge <topic> <topic> ..\n\n" <<
    "E.g.: image_bridge /camera/front/image_raw" << std::endl;
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
  auto node_ = rclcpp::Node::make_shared("ros_ign_image");
  auto it_node = std::make_shared<image_transport::ImageTransport>(node_);

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::vector<std::shared_ptr<Handler>> handlers;

  auto args = rclcpp::remove_ros_arguments(argc, argv);

  // Create publishers and subscribers
  for (auto topic : args) {
    handlers.push_back(std::make_shared<Handler>(topic, it_node, ign_node));
  }

  // Spin ROS and Ign until shutdown
  rclcpp::spin(node_);

  ignition::transport::waitForShutdown();

  return 0;
}
