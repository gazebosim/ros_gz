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

#include "ros_gz_sim/delete_entity.hpp"

#include <iostream>
#include <memory>
#include <string>

#include <CLI/CLI.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>

int main(int argc, char **argv)
{
   // Initialize ROS
  rclcpp::init(argc, argv);

   // Setup CLI11 app with description
  CLI::App app{"Delete entity from Gazebo simulation"};

   // Entity identification options (mutually exclusive)
  std::string entity_name;
  int entity_id = 0;
  auto name_option =
    app.add_option("--name", entity_name, "Name of the entity to delete");
  auto id_option =
    app.add_option("--id", entity_id, "ID of the entity to delete");
  name_option->excludes(id_option);
  id_option->excludes(name_option);

   // Entity type option
  int entity_type = ros_gz_interfaces::msg::Entity::MODEL;
  app.add_option("--type", entity_type,
                   "Entity type: 0=NONE, 1=LIGHT, 2=MODEL(default), 3=LINK, "
                   "4=VISUAL, 5=COLLISION, 6=SENSOR, 7=JOINT");

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

   // Create deleter and call service
  auto deleter = std::make_shared<EntityDeleter>();
  bool result = deleter->delete_entity(entity_name, entity_id, entity_type);

  rclcpp::shutdown();
  return result ? 0 : 1;
}
