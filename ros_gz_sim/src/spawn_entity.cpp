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

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include <CLI/CLI.hpp>

using namespace std::chrono_literals;

// Helper function to convert Euler angles to quaternion components
void euler_to_quaternion(
  double roll, double pitch, double yaw,
  double &qx, double &qy, double &qz, double &qw)
  {
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

class EntitySpawner : public rclcpp::Node {
public:
  EntitySpawner()
  : Node("entity_spawner")
  {
    client_ = create_client<ros_gz_interfaces::srv::SpawnEntity>(
        "/world/default/create");
  }

  bool spawn_entity(
    const std::string &model_name,
    const std::string &sdf_filename,
    const geometry_msgs::msg::Pose &pose)
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
      std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    request->entity_factory.name = model_name;
    request->entity_factory.sdf_filename = sdf_filename;
    request->entity_factory.pose = pose;

    RCLCPP_INFO(this->get_logger(), "Spawning model: %s from %s",
                model_name.c_str(), sdf_filename.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w);

    // Send the request
    auto future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s",
                  response->success ? "true" : "false");

      if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn entity");
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

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Setup CLI11 app
  CLI::App app{"Spawn entity in Gazebo simulation"};

  // Required parameters
  std::string model_name;
  std::string sdf_filename;
  app.add_option("--name", model_name, "Name of the model")->required();
  app.add_option("--sdf_filename", sdf_filename, "Path to the SDF file")
    ->required();

  // Position parameters (optional)
  std::vector<double> position;
  app.add_option("--pos", position, "Position as X Y Z")->expected(3);

  // Orientation parameters (optional, mutually exclusive)
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

  // Set up geometry_msgs::msg::Pose with default values
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  // Parse and catch any CLI errors
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  // Apply position if provided
  if (!position.empty()) {
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
  }

  // Apply orientation if provided
  if (!quaternion.empty()) {
    pose.orientation.x = quaternion[0];
    pose.orientation.y = quaternion[1];
    pose.orientation.z = quaternion[2];
    pose.orientation.w = quaternion[3];
  } else if (!euler.empty()) {
    double qx, qy, qz, qw;
    euler_to_quaternion(euler[0], euler[1], euler[2], qx, qy, qz, qw);
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
  }

  // Create spawner and call service
  auto spawner = std::make_shared<EntitySpawner>();
  bool result = spawner->spawn_entity(model_name, sdf_filename, pose);

  rclcpp::shutdown();
  return result ? 0 : 1;
 }
