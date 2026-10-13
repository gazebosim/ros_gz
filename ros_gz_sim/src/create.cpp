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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <set>
#include <vector>

#include <CLI/CLI.hpp>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/scope_exit.hpp>
#include <std_msgs/msg/string.hpp>

// ROS interface for spawning entities into Gazebo.
// Suggested for use with roslaunch and loading entities from ROS param.
// If these are not needed, just use the `gz service` command line instead.

// Command line flags, used when ROS parameters aren't set.
struct CommandLineFlags
{
  std::string world;
  std::string file;
  std::string param;
  std::string string;
  std::string topic;
  std::string name;
  bool allow_renaming{false};
  double x{0};
  double y{0};
  double z{0};
  double R{0};
  double P{0};
  double Y{0};
};

// Rewrite the gflags syntax this tool historically accepted (`-flag value`,
// `--flag value`, `-flag=value`, a bare `-allow_renaming`, `-noallow_renaming`
// and `-help`/`--helpshort`) into `--flag=value`, which CLI11 parses unambiguously,
// including negative numbers such as `-x -1.5`.
std::vector<std::string> normalize_gflags_arguments(const std::vector<std::string> & args)
{
  static const std::set<std::string> kValueFlags{
    "world", "file", "param", "string", "topic", "name", "x", "y", "z", "R", "P", "Y"};

  std::vector<std::string> normalized;
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string & arg = args[i];
    if (arg.size() < 2 || arg[0] != '-') {
      normalized.push_back(arg);
      continue;
    }
    const std::string body = arg.substr(arg.rfind("--", 0) == 0 ? 2 : 1);
    const auto eq = body.find('=');
    const std::string key = body.substr(0, eq);
    const bool has_value = eq != std::string::npos;

    if (kValueFlags.count(key) > 0) {
      if (has_value) {
        normalized.push_back("--" + body);
      } else if (i + 1 < args.size()) {
        normalized.push_back("--" + key + "=" + args[++i]);
      } else {
        normalized.push_back("--" + key);
      }
    } else if (key == "allow_renaming") {
      normalized.push_back(has_value ? "--" + body : "--allow_renaming=true");
    } else if (key == "noallow_renaming" && !has_value) {
      normalized.push_back("--allow_renaming=false");
    } else if (key == "help" || key == "helpshort") {
      normalized.push_back("--help");
    } else {
      normalized.push_back(arg);
    }
  }
  return normalized;
}

// Utility Function to avoid code duplication

bool set_XML_from_topic(
  const std::string & topic_name, const rclcpp::Node::SharedPtr ros2_node,
  gz::msgs::EntityFactory & req)
{
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
    topic_name, rclcpp::QoS(1).transient_local(), fun);

  rclcpp::FutureReturnCode future_ret;
  do {
    RCLCPP_INFO(ros2_node->get_logger(), "Waiting messages on topic [%s].", topic_name.c_str());
    future_ret = executor.spin_until_future_complete(xml_future, timeout);
  } while (rclcpp::ok() && future_ret != rclcpp::FutureReturnCode::SUCCESS);

  if (future_ret == rclcpp::FutureReturnCode::SUCCESS) {
    req.set_sdf(xml_future.get());
    return true;
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Failed to get XML from topic [%s].", topic_name.c_str());
    return false;
  }
}

int main(int _argc, char ** _argv)
{
  auto filtered_arguments = rclcpp::init_and_remove_ros_arguments(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_gz_sim");

  // Parse the arguments that weren't consumed by ROS
  CommandLineFlags flags;
  CLI::App app{
    "Spawn an entity in Gazebo. ROS parameters take precedence over these flags.", "create"};
  app.add_option("--world", flags.world, "World name.");
  app.add_option("--file", flags.file, "Load XML from a file.");
  app.add_option("--param", flags.param, "Load XML from a ROS param.");
  app.add_option("--string", flags.string, "Load XML from a string.");
  app.add_option("--topic", flags.topic, "Load XML from a ROS string publisher.");
  app.add_option("--name", flags.name, "Name for spawned entity.");
  app.add_option("--allow_renaming", flags.allow_renaming, "Rename entity if name already used.");
  app.add_option("--x", flags.x, "X component of initial position, in meters.");
  app.add_option("--y", flags.y, "Y component of initial position, in meters.");
  app.add_option("--z", flags.z, "Z component of initial position, in meters.");
  app.add_option("--R", flags.R, "Roll component of initial orientation, in radians.");
  app.add_option("--P", flags.P, "Pitch component of initial orientation, in radians.");
  app.add_option("--Y", flags.Y, "Yaw component of initial orientation, in radians.");

  std::vector<std::string> cli_args = normalize_gflags_arguments(
    std::vector<std::string>(filtered_arguments.begin() + 1, filtered_arguments.end()));
  // CLI11 parses a vector in reverse order
  std::reverse(cli_args.begin(), cli_args.end());
  try {
    app.parse(cli_args);
  } catch (const CLI::ParseError & e) {
    rclcpp::shutdown();
    return app.exit(e);
  }

  // Declare ROS 2 parameters to be passed from launch file
  ros2_node->declare_parameter("world", "");
  ros2_node->declare_parameter("file", "");
  ros2_node->declare_parameter("string", "");
  ros2_node->declare_parameter("topic", "");
  ros2_node->declare_parameter("name", "");
  ros2_node->declare_parameter("allow_renaming", false);
  ros2_node->declare_parameter("x", static_cast<double>(0));
  ros2_node->declare_parameter("y", static_cast<double>(0));
  ros2_node->declare_parameter("z", static_cast<double>(0));
  ros2_node->declare_parameter("R", static_cast<double>(0));
  ros2_node->declare_parameter("P", static_cast<double>(0));
  ros2_node->declare_parameter("Y", static_cast<double>(0));

  auto always_shutdown = rcpputils::make_scope_exit(
    []() {rclcpp::shutdown();});

  // World
  std::string world_name = ros2_node->get_parameter("world").as_string();
  if (world_name.empty() && !flags.world.empty()) {
    world_name = flags.world;
  }
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
    RCLCPP_WARN(
      ros2_node->get_logger(),
      "World name was not provided. Using [%s] as the default world.",
      world_name.c_str());
  }
  std::string service{"/world/" + world_name + "/create"};

  // Request message
  gz::msgs::EntityFactory req;

  // Get ROS parameters
  std::string file_name = ros2_node->get_parameter("file").as_string();
  std::string xml_string = ros2_node->get_parameter("string").as_string();
  std::string topic_name = ros2_node->get_parameter("topic").as_string();
  // Check for the SDF filename or XML string or topic name
  if (!file_name.empty()) {
    req.set_sdf_filename(file_name);
  } else if (!xml_string.empty()) {
    req.set_sdf(xml_string);
  } else if (!topic_name.empty()) {
    // set XML string by fetching it from the given topic
    if (!set_XML_from_topic(topic_name, ros2_node, req)) {
      return -1;
    }
  } else if (filtered_arguments.size() > 1) {
    // Revert to command line flags, if ROS parameters aren't specified
    // File
    if (!flags.file.empty()) {
      req.set_sdf_filename(flags.file);
    } else if (!flags.param.empty()) {  // Param
      ros2_node->declare_parameter<std::string>(flags.param);
      std::string xmlStr;
      if (ros2_node->get_parameter(flags.param, xmlStr)) {
        req.set_sdf(xmlStr);
      } else {
        RCLCPP_ERROR(
          ros2_node->get_logger(), "Failed to get XML from param [%s].", flags.param.c_str());
        return -1;
      }
    } else if (!flags.string.empty()) {  // string
      req.set_sdf(flags.string);
    } else if (!flags.topic.empty()) {  // topic
      // set XML string by fetching it from the given topic
      if (!set_XML_from_topic(flags.topic, ros2_node, req)) {
        return -1;
      }
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Must specify either -file, -param, -string or -topic");
      return -1;
    }
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Must specify either file, string or topic as ROS 2 parameters");
    return -1;
  }

  // Pose
  double x_coords = ros2_node->get_parameter("x").as_double();
  double y_coords = ros2_node->get_parameter("y").as_double();
  double z_coords = ros2_node->get_parameter("z").as_double();
  double roll = ros2_node->get_parameter("R").as_double();
  double pitch = ros2_node->get_parameter("P").as_double();
  double yaw = ros2_node->get_parameter("Y").as_double();

  flags.x = (x_coords != 0.0) ? x_coords : flags.x;
  flags.y = (y_coords != 0.0) ? y_coords : flags.y;
  flags.z = (z_coords != 0.0) ? z_coords : flags.z;
  flags.R = (roll != 0.0) ? roll : flags.R;
  flags.P = (pitch != 0.0) ? pitch : flags.P;
  flags.Y = (yaw != 0.0) ? yaw : flags.Y;
  gz::math::Pose3d pose{flags.x, flags.y, flags.z, flags.R, flags.P, flags.Y};
  gz::msgs::Set(req.mutable_pose(), pose);

  // Name
  std::string entity_name = ros2_node->get_parameter("name").as_string();
  if (!entity_name.empty()) {
    req.set_name(entity_name);
  } else {
    req.set_name(flags.name);
  }

  // Allow Renaming
  bool allow_renaming = ros2_node->get_parameter("allow_renaming").as_bool();
  req.set_allow_renaming((allow_renaming || flags.allow_renaming));

  // Request
  gz::transport::Node node;
  gz::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;

  while(rclcpp::ok()) {
    if (node.Request(service, req, timeout, rep, result)) {
      if (result && rep.data()) {
        RCLCPP_INFO(ros2_node->get_logger(), "Entity creation successful.");
        return 0;
      } else {
        RCLCPP_ERROR(
          ros2_node->get_logger(), "Entity creation failed.\n %s",
          req.DebugString().c_str());
        return 1;
      }
    } else {
      RCLCPP_WARN(
        ros2_node->get_logger(), "Waiting for service [%s] to become available ...",
        service.c_str());
    }
  }
  RCLCPP_INFO(ros2_node->get_logger(), "Entity creation was interrupted.");
  return 0;
}
