// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gflags/gflags.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <sstream>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


// ROS interface for spawning entities into Gazebo.
// Suggested for use with roslaunch and loading entities from ROS param.
// If these are not needed, just use the `gz service` command line instead.
int main(int _argc, char ** _argv)
{
  rclcpp::init(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_gz_sim");

  std::string world_prm,file_prm,param_prm,string_prm,topic_prm,name_prm;
  bool allow_renaming_prm;
  double x_prm,y_prm,z_prm,R_prm,P_prm,Y_prm;

  // declaring ros param 
  ros2_node->declare_parameter("world","");
  ros2_node->declare_parameter("file","");
  ros2_node->declare_parameter("param","");
  ros2_node->declare_parameter("string", "");
  ros2_node->declare_parameter("topic", "");
  ros2_node->declare_parameter("name", "");
  ros2_node->declare_parameter("allow_renaming", false);
  ros2_node->declare_parameter("x", 0.0);
  ros2_node->declare_parameter("y", 0.0);
  ros2_node->declare_parameter("z", 0.0);
  ros2_node->declare_parameter("R", 0.0);
  ros2_node->declare_parameter("P", 0.0);
  ros2_node->declare_parameter("Y", 0.0);

  // getting value of ros param (from param server or command line)
  ros2_node->get_parameter("world", world_prm);
  ros2_node->get_parameter("file", file_prm);
  ros2_node->get_parameter("param", param_prm);
  ros2_node->get_parameter("string", string_prm);
  ros2_node->get_parameter("topic", topic_prm);
  ros2_node->get_parameter("name", name_prm);
  ros2_node->get_parameter("allow_renaming", allow_renaming_prm);
  ros2_node->get_parameter("x", x_prm);
  ros2_node->get_parameter("y", y_prm);
  ros2_node->get_parameter("z", z_prm);
  ros2_node->get_parameter("R", R_prm);
  ros2_node->get_parameter("P", P_prm);
  ros2_node->get_parameter("Y", Y_prm);
  
  // World
  std::string world_name = world_prm;
  if (world_name.empty()) {
    // If caller doesn't provide a world name, get list of worlds from gz-sim server
    gz::transport::Node node;

    bool executed{false};
    bool result{false};
    unsigned int timeout{5000};
    std::string service{"/gazebo/worlds"};
    gz::msgs::StringMsg_V worlds_msg;

    // This loop is here to allow the server time to download resources.
    while (rclcpp::ok() && !executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requesting list of world names.");
      executed = node.Request(service, timeout, worlds_msg, result);
    }

    if (!executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Timed out when getting world names.");
      return -1;
    }

    if (!result || worlds_msg.data().empty()) {
      RCLCPP_INFO(ros2_node->get_logger(), "Failed to get world names.");
      return -1;
    }

    world_name = worlds_msg.data(0);
  }
  std::string service{"/world/" + world_name + "/create"};

  // Request message
  gz::msgs::EntityFactory req;

  // File
  if (!file_prm.empty()) {
    req.set_sdf_filename(file_prm);
  } else if (!param_prm.empty()) {  // Param
    ros2_node->declare_parameter<std::string>(param_prm);

    std::string xmlStr;
    if (ros2_node->get_parameter(param_prm, xmlStr)) {
      req.set_sdf(xmlStr);
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed to get XML from param [%s].", param_prm.c_str());
      return -1;
    }
  } else if (!string_prm.empty()) {  // string
    req.set_sdf(string_prm);
  } else if (!topic_prm.empty()) {  // topic
    const auto timeout = std::chrono::seconds(1);
    std::promise<std::string> xml_promise;
    std::shared_future<std::string> xml_future(xml_promise.get_future());

    std::function<void(const std_msgs::msg::String::SharedPtr)> fun =
      [&xml_promise](const std_msgs::msg::String::SharedPtr msg) {
        xml_promise.set_value(msg->data);
      };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ros2_node);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subs;
    // Transient local is similar to latching in ROS 1.
    description_subs = ros2_node->create_subscription<std_msgs::msg::String>(
      topic_prm, rclcpp::QoS(1).transient_local(), fun);

    rclcpp::FutureReturnCode future_ret;
    do {
      RCLCPP_INFO(ros2_node->get_logger(), "Waiting messages on topic [%s].", topic_prm.c_str());
      future_ret = executor.spin_until_future_complete(xml_future, timeout);
    } while (rclcpp::ok() && future_ret != rclcpp::FutureReturnCode::SUCCESS);

    if (future_ret == rclcpp::FutureReturnCode::SUCCESS) {
      req.set_sdf(xml_future.get());
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed to get XML from topic [%s].", topic_prm.c_str());
      return -1;
    }
  } else {
    RCLCPP_ERROR(ros2_node->get_logger(), "Must specify either -file, -param, -stdin or -topic");
    return -1;
  }

  // Pose
  gz::math::Pose3d pose{x_prm, y_prm, z_prm, R_prm, P_prm, Y_prm};
  gz::msgs::Set(req.mutable_pose(), pose);

  // Name
  if (!name_prm.empty()) {
    req.set_name(name_prm);
  }

  if (allow_renaming_prm) {
    req.set_allow_renaming(allow_renaming_prm);
  }

  // Request
  gz::transport::Node node;
  gz::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;
  bool executed = node.Request(service, req, timeout, rep, result);

  if (executed) {
    if (result && rep.data()) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requested creation of entity.");
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed request to create entity.\n %s",
        req.DebugString().c_str());
    }
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Request to create entity from service [%s] timed out..\n %s",
      service.c_str(), req.DebugString().c_str());
  }
  RCLCPP_INFO(ros2_node->get_logger(), "OK creation of entity.");

  return 0;
}
