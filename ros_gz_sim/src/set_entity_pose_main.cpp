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

#include "ros_gz_sim/set_entity_pose.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Setup CLI11 app with description
  CLI::App app{"Set entity pose in Gazebo simulation"};

  // Entity identification options
  std::string entity_name;
  int entity_id = 0;
  auto name_option =
    app.add_option("--name", entity_name, "Name of the entity");
  auto id_option = app.add_option("--id", entity_id, "ID of the entity");
  name_option->excludes(id_option);
  id_option->excludes(name_option);

  // Entity type option
  int entity_type = ros_gz_interfaces::msg::Entity::MODEL;
  app.add_option("--type", entity_type,
                   "Entity type: 0=NONE, 1=LIGHT, 2=MODEL(default), 3=LINK, "
                   "4=VISUAL, 5=COLLISION, 6=SENSOR, 7=JOINT");

  // Position parameters
  std::vector<double> position = {0.0, 0.0, 0.0};
  app.add_option("--pos", position, "Position as X Y Z")->expected(3);

  // Orientation parameters
  std::vector<double> quaternion;
  std::vector<double> euler;
  auto quat_option =
    app.add_option("--quat", quaternion, "Orientation as quaternion X Y Z W")
    ->expected(4);
  auto euler_option =
    app.add_option("--euler", euler,
                    "Orientation as Euler angles ROLL PITCH YAW (in radians)")
    ->expected(3);
  quat_option->excludes(euler_option);
  euler_option->excludes(quat_option);

  // Parse and catch any CLI errors
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  // Ensure either name or ID is provided
  if (entity_name.empty() && entity_id <= 0) {
    std::cerr << "Error: Either --name or --id must be provided" << std::endl;
    std::cout << app.help() << std::endl;
    return 1;
  }

  // Set defaults for position if not provided
  double x = 0.0, y = 0.0, z = 0.0;
  if (!position.empty()) {
    x = position[0];
    y = position[1];
    z = position[2];
  }

  // Set defaults for orientation
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  bool use_quaternion = true;

  // Apply orientation if provided
  if (!quaternion.empty()) {
    qx = quaternion[0];
    qy = quaternion[1];
    qz = quaternion[2];
    qw = quaternion[3];
    use_quaternion = true;
  } else if (!euler.empty()) {
    qx = euler[0];    // roll
    qy = euler[1];    // pitch
    qz = euler[2];    // yaw
    use_quaternion = false;
  }

  // Create pose setter and call service
  auto pose_setter = std::make_shared<EntityPoseSetter>();
  bool result =
    pose_setter->set_entity_pose(entity_name, entity_id, entity_type, x, y, z,
                                  qx, qy, qz, qw, use_quaternion);

  rclcpp::shutdown();
  return result ? 0 : 1;
}
