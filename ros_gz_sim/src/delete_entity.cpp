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

 #include <chrono>
 #include <iostream>
 #include <memory>
 #include <string>

 #include <rclcpp/rclcpp.hpp>
 #include <ros_gz_interfaces/msg/entity.hpp>
 #include <ros_gz_interfaces/srv/delete_entity.hpp>
 #include <CLI/CLI.hpp>

using namespace std::chrono_literals;

 // Default constructor
EntityDeleter::EntityDeleter()
: Node("entity_deleter")
{
  client_ = create_client<ros_gz_interfaces::srv::DeleteEntity>(
     "/world/default/remove");
}

 // Constructor with external client.
EntityDeleter::EntityDeleter(
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr client)
: Node("entity_deleter"), client_(client) {}

 // Implementation of the delete_entity method
bool EntityDeleter::delete_entity(
  const std::string & entity_name, int entity_id,
  int entity_type)
{
   // Wait for the service to be available
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
  }

   // Create the request
  auto request =
    std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
  auto entity = ros_gz_interfaces::msg::Entity();

   // Set entity identification (name or ID)
  if (entity_id > 0) {
    entity.id = entity_id;
    RCLCPP_INFO(this->get_logger(), "Deleting entity with ID: %d", entity_id);
  } else if (!entity_name.empty()) {
    entity.name = entity_name;
    RCLCPP_INFO(this->get_logger(), "Deleting entity with name: %s",
                 entity_name.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(),
                   "Either entity name or ID must be provided");
    return false;
  }

  entity.type = entity_type;
  request->entity = entity;

   // Send the request
  auto future = client_->async_send_request(request);

   // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Result: %s",
                 response->success ? "true" : "false");

    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to delete entity");
      return false;
    }
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    return false;
  }
}

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
  int entity_type = 6;     // Default to MODEL type
  app.add_option("--type", entity_type,
                   "Entity type: 0=NONE, 1=LIGHT, 2=LINK, 3=VISUAL, 4=COLLISION, "
                   "5=SENSOR, 6=MODEL(default)");

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
