#include <memory>
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"

using namespace std::chrono_literals;

class EntitySpawner : public rclcpp::Node
{
public:
  EntitySpawner()
  : Node("entity_spawner")
  {
    client_ = create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
  }

  bool spawn_entity(const std::string & model_name, const std::string & sdf_filename)
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
    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    request->entity_factory.name = model_name;
    request->entity_factory.sdf_filename = sdf_filename;

    RCLCPP_INFO(
      this->get_logger(), "Spawning model: %s from %s",
      model_name.c_str(), sdf_filename.c_str());

    // Send the request
    auto future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", response->success ? "true" : "false");
      
      if (!response->success) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to spawn entity");
        return false;
      }
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return false;
    }
  }

private:
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: spawn_entity <model_name> <sdf_filename>" << std::endl;
    std::cerr << "Example: spawn_entity cardboard_box /path/to/models/cardboard_box/model.sdf" << std::endl;
    return 1;
  }

  std::string model_name = argv[1];
  std::string sdf_filename = argv[2];

  auto spawner = std::make_shared<EntitySpawner>();
  bool result = spawner->spawn_entity(model_name, sdf_filename);

  rclcpp::shutdown();
  return result ? 0 : 1;
}