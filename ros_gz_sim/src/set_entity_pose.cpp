/*
 * Copyright 2025 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <CLI/CLI.hpp>

using namespace std::chrono_literals;

// Utility function to convert Euler angles to Quaternion
geometry_msgs::msg::Quaternion euler_to_quaternion(
  double roll, double pitch, double yaw)
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

class EntityPoseSetter : public rclcpp::Node {
public:
  EntityPoseSetter()
  : Node("entity_pose_setter")
  {
    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
        "/world/default/set_pose");
  }

  bool set_entity_pose(
    const std::string & entity_name, int entity_id,
    int entity_type, double x, double y, double z, double qx,
    double qy, double qz, double qw,
    bool use_quaternion = true)
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
      std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    auto entity = ros_gz_interfaces::msg::Entity();

    // Set entity identification (name or ID)
    if (entity_id > 0) {
      entity.id = entity_id;
      RCLCPP_INFO(this->get_logger(), "Setting pose for entity with ID: %d",
                  entity_id);
    } else if (!entity_name.empty()) {
      entity.name = entity_name;
      RCLCPP_INFO(this->get_logger(), "Setting pose for entity with name: %s",
                  entity_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Either entity name or ID must be provided");
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
      qw = pose.orientation.w;
    }

    request->pose = pose;

    RCLCPP_INFO(this->get_logger(), "Position: x=%f, y=%f, z=%f",
                pose.position.x, pose.position.y, pose.position.z);

    RCLCPP_INFO(this->get_logger(), "Orientation: x=%f, y=%f, z=%f, w=%f",
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w);

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

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Setup CLI11 app with description
  CLI::App app{"Set entity pose in Gazebo simulation"};

  // Entity identification options (mutually exclusive)
  std::string entity_name;
  int entity_id = 0;
  auto name_option =
    app.add_option("--name", entity_name, "Name of the entity");
  auto id_option = app.add_option("--id", entity_id, "ID of the entity");
  name_option->excludes(id_option);
  id_option->excludes(name_option);

  // Entity type option
  int entity_type = 6;    // Default to MODEL type
  app.add_option("--type", entity_type,
                 "Entity type: 0=NONE, 1=LIGHT, 2=LINK, 3=VISUAL, 4=COLLISION, "
                 "5=SENSOR, 6=MODEL(default)");

  // Position parameters
  std::vector<double> position = {0.0, 0.0, 0.0};
  app.add_option("--pos", position, "Position as X Y Z")->expected(3);

  // Orientation parameters (mutually exclusive)
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

  // Add validators and requirements
  // Parse and catch any CLI errors
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  // Manual validation: Ensure either name or ID is provided
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
