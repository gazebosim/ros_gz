#include <memory>
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"

using namespace std::chrono_literals;

class EntityDeleter : public rclcpp::Node
{
public:
  EntityDeleter()
  : Node("entity_deleter")
  {
    client_ = create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/default/remove");
  }

  bool delete_entity(const std::string & entity_name, int entity_id, int entity_type)
  {
    // Wait for the service to be available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    // Create the request
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    auto entity = ros_gz_interfaces::msg::Entity();
    
    // Set entity identification (name or ID)
    if (entity_id > 0) {
      entity.id = entity_id;
      RCLCPP_INFO(this->get_logger(), "Deleting entity with ID: %d", entity_id);
    } else if (!entity_name.empty()) {
      entity.name = entity_name;
      RCLCPP_INFO(this->get_logger(), "Deleting entity with name: %s", entity_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Either entity name or ID must be provided");
      return false;
    }
    
    entity.type = entity_type;
    request->entity = entity;

    // Send the request
    auto future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", response->success ? "true" : "false");
      
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

private:
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr client_;
};

void print_usage()
{
  std::cout << "Usage: delete_entity [--name NAME | --id ID] [--type TYPE]" << std::endl;
  std::cout << "Examples:" << std::endl;
  std::cout << "  delete_entity --name cardboard_box" << std::endl;
  std::cout << "  delete_entity --id 8" << std::endl;
  std::cout << "  delete_entity --name cardboard_box --type 6" << std::endl;
  std::cout << std::endl;
  std::cout << "Entity type values:" << std::endl;
  std::cout << "  0: NONE" << std::endl;
  std::cout << "  1: LIGHT" << std::endl;
  std::cout << "  2: LINK" << std::endl;
  std::cout << "  3: VISUAL" << std::endl;
  std::cout << "  4: COLLISION" << std::endl;
  std::cout << "  5: SENSOR" << std::endl;
  std::cout << "  6: MODEL (default)" << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    print_usage();
    return 1;
  }

  // Parse command line arguments
  std::string entity_name = "";
  int entity_id = 0;
  int entity_type = 6;  // Default to MODEL type

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    
    if (arg == "--name" && i + 1 < argc) {
      entity_name = argv[++i];
    } else if (arg == "--id" && i + 1 < argc) {
      try {
        entity_id = std::stoi(argv[++i]);
      } catch (const std::exception & e) {
        std::cerr << "Error: ID must be an integer" << std::endl;
        return 1;
      }
    } else if (arg == "--type" && i + 1 < argc) {
      try {
        entity_type = std::stoi(argv[++i]);
      } catch (const std::exception & e) {
        std::cerr << "Error: Type must be an integer" << std::endl;
        return 1;
      }
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      print_usage();
      return 1;
    }
  }

  if (entity_name.empty() && entity_id == 0) {
    std::cerr << "Error: Either --name or --id must be provided" << std::endl;
    print_usage();
    return 1;
  }

  auto deleter = std::make_shared<EntityDeleter>();
  bool result = deleter->delete_entity(entity_name, entity_id, entity_type);

  rclcpp::shutdown();
  return result ? 0 : 1;
}