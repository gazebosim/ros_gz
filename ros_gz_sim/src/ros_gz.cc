/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include <filesystem>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>
#include <ros_gz_bridge/bridge_config.hpp>

#include "ros_gz.hh"

using namespace gz;
using namespace ros_gz_sim;

/// \brief Private ROSGzPlugin data class.
class ROSGzPlugin::Implementation
{
  /// \brief The ROS 2 <--> Gz bridge.
  public: std::shared_ptr<ros_gz_bridge::RosGzBridge> bridge;

  /// \brief The ROS 2 executor.
  public: std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;

  /// \brief A thread to call spin and not block the Gazebo thread.
  public: std::thread thread;
};

//////////////////////////////////////////////////
ROSGzPlugin::ROSGzPlugin()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
ROSGzPlugin::~ROSGzPlugin()
{
  this->dataPtr->exec->cancel();
  this->dataPtr->thread.join();
}

//////////////////////////////////////////////////
void ROSGzPlugin::Configure(const sim::Entity &/*_entity*/,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &/*_ecm*/, sim::EventManager &/*_eventMgr*/)
{
  // Ensure that ROS is setup.
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  if (!_sdf->HasElement("config_file"))
  {
    gzerr << "No <config_file> found. Plugin disabled." << std::endl;
    return;
  }

  // Sanity check: Make sure that the config file exists and it's a file.
  std::filesystem::path configFile = _sdf->Get<std::string>("config_file");
  if (!std::filesystem::is_regular_file(configFile))
  {
    gzerr << "[" << configFile << "] is not a regular file. Plugin disabled"
          << std::endl;
    return;
  }

  // Create the bridge passing the parameters as rclcpp::NodeOptions().
  this->dataPtr->bridge = std::make_shared<ros_gz_bridge::RosGzBridge>(
    rclcpp::NodeOptions().append_parameter_override("config_file", configFile));

  // Create the executor.
  this->dataPtr->exec =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->exec->add_node(this->dataPtr->bridge);

  // Spin in a separate thread to not block Gazebo.
  this->dataPtr->thread = std::thread([this](){this->dataPtr->exec->spin();});
}

GZ_ADD_PLUGIN(ROSGzPlugin,
              sim::System,
              ROSGzPlugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(ros_gz_sim::ROSGzPlugin,
                    "ros_gz_sim::ROSGzPlugin")
