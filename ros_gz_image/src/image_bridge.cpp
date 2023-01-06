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


#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ros_gz_bridge/convert.hpp>

//////////////////////////////////////////////////
/// \brief Bridges one topic
class Handler
{
public:
  /// \brief Constructor
  /// \param[in] _topic Image base topic
  /// \param[in] _it_node Pointer to image transport node
  /// \param[in] _gz_node Pointer to Gazebo node
  Handler(
    const std::string & _topic,
    std::shared_ptr<image_transport::ImageTransport> _it_node,
    std::shared_ptr<gz::transport::Node> _gz_node)
  {
    this->ros_pub = _it_node->advertise(_topic, 1);

    _gz_node->Subscribe(_topic, &Handler::OnImage, this);
  }

private:
  /// \brief Callback when Gazebo image is received
  /// \param[in] _gz_msg Gazebo message
  void OnImage(const gz::msgs::Image & _gz_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
    this->ros_pub.publish(ros_msg);
  }

  /// \brief ROS image publisher
  image_transport::Publisher ros_pub;
};

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Bridge a collection of Gazebo Transport image topics to ROS " <<
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
  auto node_ = rclcpp::Node::make_shared("ros_gz_image");
  auto it_node = std::make_shared<image_transport::ImageTransport>(node_);

  // Gazebo node
  auto gz_node = std::make_shared<gz::transport::Node>();

  std::vector<std::shared_ptr<Handler>> handlers;

  // skip the process name in argument procesing
  ++argv;
  --argc;
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  // Create publishers and subscribers
  for (auto topic : args) {
    handlers.push_back(std::make_shared<Handler>(topic, it_node, gz_node));
  }

  // Spin ROS and Gz until shutdown
  rclcpp::spin(node_);

  return 0;
}
