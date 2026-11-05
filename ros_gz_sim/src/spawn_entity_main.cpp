// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include "ros_gz_sim/spawn_entity.hpp"

#include <exception>
#include <iostream>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

 /// \brief Main entry point for the entity spawner node.
 /// \details Parses arguments, constructs a pose, and calls the spawn service.
 /// \param[in] argc Number of command-line arguments.
 /// \param[in] argv List of command-line arguments.
 /// \return Exit status code: 0 for success, 1 for failure.
int main(int argc, char **argv)
{
   // Initialize ROS
  rclcpp::init(argc, argv);

  try {
     // Parse command line arguments
    CommandLineArgs args = parse_arguments(argc, argv);
     // Set up geometry_msgs::msg::Pose with default values
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

     // Apply position if provided
    if (!args.position.empty()) {
      pose.position.x = args.position[0];
      pose.position.y = args.position[1];
      pose.position.z = args.position[2];
    }

     // Apply orientation if provided
    if (!args.quaternion.empty()) {
      pose.orientation.x = args.quaternion[0];
      pose.orientation.y = args.quaternion[1];
      pose.orientation.z = args.quaternion[2];
      pose.orientation.w = args.quaternion[3];
    } else if (!args.euler.empty()) {
       // Use tf2 for Euler to quaternion conversion
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(args.euler[0], args.euler[1], args.euler[2]);

       // Copy to pose orientation
      pose.orientation.x = tf2_quat.x();
      pose.orientation.y = tf2_quat.y();
      pose.orientation.z = tf2_quat.z();
      pose.orientation.w = tf2_quat.w();
    }

     // Create spawner and call service
    auto spawner = std::make_shared<EntitySpawner>();
    bool result =
      spawner->spawn_entity(args.model_name, args.sdf_filename, pose);

    rclcpp::shutdown();
    return result ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
