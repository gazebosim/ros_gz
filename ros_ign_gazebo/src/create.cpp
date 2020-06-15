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
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

DEFINE_string(world, "", "World name.");
DEFINE_string(file, "", "Load XML from a file.");
DEFINE_string(param, "", "Load XML from a ROS param.");
DEFINE_string(string, "", "Load XML from a string.");
DEFINE_string(name, "", "Name for spawned entity.");
DEFINE_bool(allow_renaming, false, "Rename entity if name already used.");
DEFINE_double(x, 0, "X component of initial position, in meters.");
DEFINE_double(y, 0, "Y component of initial position, in meters.");
DEFINE_double(z, 0, "Z component of initial position, in meters.");
DEFINE_double(R, 0, "Roll component of initial orientation, in radians.");
DEFINE_double(P, 0, "Pitch component of initial orientation, in radians.");
DEFINE_double(Y, 0, "Yaw component of initial orientation, in radians.");

// ROS interface for spawning entities into Ignition.
// Suggested for use with roslaunch and loading entities from ROS param.
// If these are not needed, just use the `ign service` command line instead.
int main(int _argc, char ** _argv)
{
  gflags::AllowCommandLineReparsing();
  gflags::SetUsageMessage(
    R"(Usage: create -world [arg] [-file FILE] [-param PARAM] [-string STRING]
                       [-name NAME] [-X X] [-Y Y] [-Z Z] [-Roll ROLL]
                       [-Pitch PITCH] [-Yaw YAW])");
  gflags::ParseCommandLineFlags(&_argc, &_argv, true);

  rclcpp::init(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_ign_gazebo");

  // World
  std::string world_name = FLAGS_world;
  if (world_name.empty()) {

    // If caller doesn't provide a world name, get list of worlds from ign-gazebo server
    ignition::transport::Node node;

    bool executed{false};
    bool result{false};
    unsigned int timeout{5000};
    std::string service{"/gazebo/worlds"};
    ignition::msgs::StringMsg_V worlds_msg;

    // This loop is here to allow the server time to download resources.
    while (!executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requesting list of world names.");
      executed = node.Request(service, timeout, worlds_msg, result);
    }

    if (!executed) {
      RCLCPP_INFO(ros2_node->get_logger(),
          "Timed out when getting world names.");
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
  ignition::msgs::EntityFactory req;

  // File
  if (!FLAGS_file.empty()) {
    req.set_sdf_filename(FLAGS_file);
  } else if (!FLAGS_param.empty()) {  // Param
    ros2_node->declare_parameter(FLAGS_param);

    std::string xmlStr;
    if (ros2_node->get_parameter(FLAGS_param, xmlStr)) {
      req.set_sdf(xmlStr);
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed to get XML from param [%s].",
        FLAGS_param.c_str());
      return -1;
    }
  } else if (!FLAGS_string.empty()) {  // string
    req.set_sdf(FLAGS_string);
  } else {
    RCLCPP_ERROR(ros2_node->get_logger(), "Must specify either -file, -param or -stdin");
    return -1;
  }

  // Pose
  ignition::math::Pose3d pose
  {
    FLAGS_x,
    FLAGS_y,
    FLAGS_z,
    FLAGS_R,
    FLAGS_P,
    FLAGS_Y
  };
  ignition::msgs::Set(req.mutable_pose(), pose);

  // Name
  if (!FLAGS_name.empty()) {
    req.set_name(FLAGS_name);
  }

  if (FLAGS_allow_renaming) {
    req.set_allow_renaming(FLAGS_allow_renaming);
  }

  // Request
  ignition::transport::Node node;
  ignition::msgs::Boolean rep;
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