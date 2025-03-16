#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

// Helper function to convert Euler angles to quaternion components
void euler_to_quaternion(double roll, double pitch, double yaw, 
                         double& qx, double& qy, double& qz, double& qw) {
  // Calculate quaternion components from Euler angles (ZYX convention)
  // Implementation based on standard rotation matrix to quaternion conversion
  
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

class EntitySpawner : public rclcpp::Node
{
public:
  EntitySpawner()
  : Node("entity_spawner")
  {
    client_ = create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
  }

  bool spawn_entity(
    const std::string & model_name, 
    const std::string & sdf_filename,
    const geometry_msgs::msg::Pose & pose)
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
    request->entity_factory.pose = pose;

    RCLCPP_INFO(
      this->get_logger(), "Spawning model: %s from %s",
      model_name.c_str(), sdf_filename.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

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

void print_usage() {
  std::cerr << "Usage: spawn_entity --name NAME --sdf_filename SDF_PATH [--pos X Y Z] [--quat X Y Z W | --euler ROLL PITCH YAW]" << std::endl;
  std::cerr << "Example: spawn_entity --name cardboard_box --sdf_filename /path/to/models/cardboard_box/model.sdf --pos 1.0 2.0 3.0 --euler 0.0 0.0 1.57" << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Default values
  std::string model_name = "";
  std::string sdf_filename = "";
  geometry_msgs::msg::Pose pose;
  
  // Default pose
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  
  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    
    if (arg == "--name" && i + 1 < argc) {
      model_name = argv[++i];
    } else if (arg == "--sdf_filename" && i + 1 < argc) {
      sdf_filename = argv[++i];
    } else if (arg == "--pos" && i + 3 < argc) {
      pose.position.x = std::stod(argv[i+1]);
      pose.position.y = std::stod(argv[i+2]);
      pose.position.z = std::stod(argv[i+3]);
      i += 3;
    } else if (arg == "--quat" && i + 4 < argc) {
      pose.orientation.x = std::stod(argv[i+1]);
      pose.orientation.y = std::stod(argv[i+2]);
      pose.orientation.z = std::stod(argv[i+3]);
      pose.orientation.w = std::stod(argv[i+4]);
      i += 4;
    } else if (arg == "--euler" && i + 3 < argc) {
      double roll = std::stod(argv[i+1]);
      double pitch = std::stod(argv[i+2]);
      double yaw = std::stod(argv[i+3]);
      
      double qx, qy, qz, qw;
      euler_to_quaternion(roll, pitch, yaw, qx, qy, qz, qw);
      pose.orientation.x = qx;
      pose.orientation.y = qy;
      pose.orientation.z = qz;
      pose.orientation.w = qw;
      
      i += 3;
    } else if (arg == "--help") {
      print_usage();
      return 0;
    }
  }
  
  // Check if required arguments are provided
  if (model_name.empty() || sdf_filename.empty()) {
    std::cerr << "Error: Model name and SDF filename are required." << std::endl;
    print_usage();
    return 1;
  }

  auto spawner = std::make_shared<EntitySpawner>();
  bool result = spawner->spawn_entity(model_name, sdf_filename, pose);

  rclcpp::shutdown();
  return result ? 0 : 1;
}