#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

// Utility function to convert Euler angles to Quaternion
geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Quaternion q;
  
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  
  return q;
}

class EntityPoseSetter : public rclcpp::Node
{
public:
  EntityPoseSetter()
  : Node("entity_pose_setter")
  {
    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/default/set_pose");
  }

  bool set_entity_pose(
    const std::string & entity_name, int entity_id, int entity_type,
    double x, double y, double z,
    double qx, double qy, double qz, double qw,
    bool use_quaternion = true)
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
    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    auto entity = ros_gz_interfaces::msg::Entity();
    
    // Set entity identification (name or ID)
    if (entity_id > 0) {
      entity.id = entity_id;
      RCLCPP_INFO(this->get_logger(), "Setting pose for entity with ID: %d", entity_id);
    } else if (!entity_name.empty()) {
      entity.name = entity_name;
      RCLCPP_INFO(this->get_logger(), "Setting pose for entity with name: %s", entity_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Either entity name or ID must be provided");
      return false;
    }
    
    entity.type = entity_type;
    request->entity = entity;
    
    // Set position
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    
    // Set orientation (quaternion)
    if (use_quaternion) {
      pose.orientation.x = qx;
      pose.orientation.y = qy;
      pose.orientation.z = qz;
      pose.orientation.w = qw;
    } else {
      // In this case, qx=roll, qy=pitch, qz=yaw (in radians)
      pose.orientation = euler_to_quaternion(qx, qy, qz);
      qw = pose.orientation.w;  // For logging
    }
    
    request->pose = pose;
    
    RCLCPP_INFO(
      this->get_logger(), "Position: x=%f, y=%f, z=%f", 
      pose.position.x, pose.position.y, pose.position.z);
    
    RCLCPP_INFO(
      this->get_logger(), "Orientation: x=%f, y=%f, z=%f, w=%f",
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
        RCLCPP_ERROR(this->get_logger(), "Failed to set entity pose");
        return false;
      }
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return false;
    }
  }

private:
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
};

void print_usage()
{
  std::cout << "Usage: set_entity_pose [--name NAME | --id ID] [--type TYPE] [--pos X Y Z] [--quat X Y Z W | --euler ROLL PITCH YAW]" << std::endl;
  std::cout << "Examples:" << std::endl;
  std::cout << "  set_entity_pose --name cardboard_box --pos 1.0 2.0 3.0" << std::endl;
  std::cout << "  set_entity_pose --id 8 --pos 1.0 2.0 3.0 --quat 0.0 0.0 0.0 1.0" << std::endl;
  std::cout << "  set_entity_pose --name cardboard_box --pos 1.0 2.0 3.0 --euler 0.0 0.0 1.57" << std::endl;
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
  double x = 0.0, y = 0.0, z = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  bool use_quaternion = true;
  bool pos_set = false;
  bool orientation_set = false;

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
    } else if (arg == "--pos" && i + 3 < argc) {
      try {
        x = std::stod(argv[++i]);
        y = std::stod(argv[++i]);
        z = std::stod(argv[++i]);
        pos_set = true;
      } catch (const std::exception & e) {
        std::cerr << "Error: Position coordinates must be numbers" << std::endl;
        return 1;
      }
    } else if (arg == "--quat" && i + 4 < argc) {
      try {
        qx = std::stod(argv[++i]);
        qy = std::stod(argv[++i]);
        qz = std::stod(argv[++i]);
        qw = std::stod(argv[++i]);
        use_quaternion = true;
        orientation_set = true;
      } catch (const std::exception & e) {
        std::cerr << "Error: Quaternion values must be numbers" << std::endl;
        return 1;
      }
    } else if (arg == "--euler" && i + 3 < argc) {
      try {
        // Here we temporarily store roll, pitch, yaw in qx, qy, qz
        qx = std::stod(argv[++i]);  // roll
        qy = std::stod(argv[++i]);  // pitch
        qz = std::stod(argv[++i]);  // yaw
        use_quaternion = false;
        orientation_set = true;
      } catch (const std::exception & e) {
        std::cerr << "Error: Euler angles must be numbers" << std::endl;
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

  auto pose_setter = std::make_shared<EntityPoseSetter>();
  bool result = pose_setter->set_entity_pose(
    entity_name, entity_id, entity_type,
    x, y, z, qx, qy, qz, qw, use_quaternion);

  rclcpp::shutdown();
  return result ? 0 : 1;
}