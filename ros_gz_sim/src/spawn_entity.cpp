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

 #include <chrono>
 #include <cmath>
 #include <iostream>
 #include <memory>
 #include <string>
 #include <vector>

 #include <CLI/CLI.hpp>
 #include <tf2/LinearMath/Quaternion.hpp>
 #include <tf2/LinearMath/Matrix3x3.hpp>

 /// \brief Parses command-line arguments using CLI11.
 /// \param[in] argc Argument count.
 /// \param[in] argv Argument vector.
 /// \return A filled CommandLineArgs struct containing model details and pose.
 /// \throws std::runtime_error if argument parsing fails.
CommandLineArgs parse_arguments(int argc, char **argv)
{
  CommandLineArgs args;

   // Setup CLI11 app
  CLI::App app{"Spawn entity in Gazebo simulation"};

   // Required parameters
  app.add_option("--name", args.model_name, "Name of the model")->required();
  app.add_option("--sdf_filename", args.sdf_filename, "Path to the SDF file")
  ->required();

   // Position parameters (optional)
  app.add_option("--pos", args.position, "Position as X Y Z")->expected(3);

   // Orientation parameters (optional, mutually exclusive)
  auto quat_option = app.add_option("--quat", args.quaternion,
                                     "Orientation as quaternion X Y Z W")
    ->expected(4);
  auto euler_option =
    app.add_option("--euler", args.euler,
                      "Orientation as Euler angles ROLL PITCH YAW (in radians)")
    ->expected(3);
  quat_option->excludes(euler_option);
  euler_option->excludes(quat_option);

   // Parse and catch any CLI errors
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    app.exit(e);
    throw std::runtime_error("Failed to parse command line arguments");
  }

  return args;
}

 /// \brief Default constructor.
 /// \details Initializes the node and creates a spawn entity service client for
 /// Gazebo.
EntitySpawner::EntitySpawner()
: Node("entity_spawner")
{
  client_ = create_client<ros_gz_interfaces::srv::SpawnEntity>(
       "/world/default/create");
}

 /// \brief Constructor with external service client.
 /// \param[in] client External spawn entity service client for dependency
 /// injection/testing.
EntitySpawner::EntitySpawner(
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client)
: Node("entity_spawner"), client_(client) {}

 /// \brief Calls the `/world/default/create` service to spawn a model in Gazebo.
 /// \param[in] model_name Name of the entity.
 /// \param[in] sdf_filename Path to the SDF file to load.
 /// \param[in] pose Initial pose of the entity in the world frame.
 /// \return True if the service call was successful and the entity was spawned;
 /// otherwise, false.
bool EntitySpawner::spawn_entity(
  const std::string & model_name,
  const std::string & sdf_filename,
  const geometry_msgs::msg::Pose & pose)
{
  using namespace std::chrono_literals;
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
    rclcpp::FutureReturnCode::SUCCESS)
  {
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
