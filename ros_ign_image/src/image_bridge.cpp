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
#include <string>

#include <ignition/transport/Node.hh>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros_ign_bridge/convert_builtin_interfaces.hpp>

//////////////////////////////////////////////////
/// \brief Bridges one topic
class Handler
{
  /// \brief Constructor
  /// \param[in] _topic Image base topic
  /// \param[in] _it_node Pointer to image transport node
  /// \param[in] _ign_node Pointer to Ignition node
  public: Handler(
      const std::string & _topic,
      std::shared_ptr<image_transport::ImageTransport> _it_node,
      std::shared_ptr<ignition::transport::Node> _ign_node)
  {
    this->ros_pub = _it_node->advertise(_topic, 1);

    _ign_node->Subscribe(_topic, &Handler::OnImage, this);
  }

  /// \brief Callback when Ignition image is received
  /// \param[in] _ign_msg Ignition message
  private: void OnImage(const ignition::msgs::Image & _ign_msg)
  {
    sensor_msgs::Image ros_msg;
    ros_ign_bridge::convert_ign_to_ros(_ign_msg, ros_msg);
    this->ros_pub.publish(ros_msg);
  }

  /// \brief ROS image publisher
  private: image_transport::Publisher ros_pub;
};

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Bridge a collection of Ignition Transport image topics to ROS "
            << "using image_transport.\n\n"
            << "  image_bridge <topic> <topic> ..\n\n"
            << "E.g.: image_bridge /camera/front/image_raw" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2)
  {
    usage();
    return -1;
  }

  ros::init(argc, argv, "ros_ign_image");

  // ROS node
  ros::NodeHandle ros_node;
  auto it_node = std::make_shared<image_transport::ImageTransport>(ros_node);

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::vector<std::shared_ptr<Handler>> handlers;

  // Create publishers and subscribers
  for (auto i = 1; i < argc; ++i)
  {
    auto topic = std::string(argv[i]);
    handlers.push_back(std::make_shared<Handler>(topic, it_node, ign_node));
  }

  // Spin ROS and Ign until shutdown
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ignition::transport::waitForShutdown();

  return 0;
}
