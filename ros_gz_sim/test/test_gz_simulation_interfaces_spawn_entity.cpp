// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <gz/common/Filesystem.hh>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/components/Namespace.hh>

#include <rclcpp/rclcpp.hpp>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>

#include "ros_gz_sim/gz_simulation_interfaces.hpp"

using namespace std::chrono_literals;

TEST(GzSimulationInterfacesSpawnEntityTest, ForwardsNamespaceToGazeboRequest)
{
  rclcpp::init(0, nullptr);
  auto ros_node = std::make_shared<rclcpp::Node>("test_spawn_entity");

  gz::sim::ServerConfig server_config;
  const std::string sdf_file = gz::common::joinPaths(
    gz::common::parentPath(__FILE__), "sdf", "gz_simulation_interfaces.sdf");
  ASSERT_TRUE(server_config.SetSdfFile(sdf_file));
  auto server = std::make_unique<gz::sim::Server>(server_config);
  server->RunOnce(true);

  auto sim_interfaces =
    std::make_unique<ros_gz_sim::gz_simulation_interfaces::GzSimulationInterfaces>(ros_node);

  server->Run(false, 0, false);

  auto client = ros_node->create_client<simulation_interfaces::srv::SpawnEntity>("spawn_entity");
  ASSERT_TRUE(client->wait_for_service(5s));
  std::string expected_namespace = "test_ns";
  auto request =
    std::make_shared<simulation_interfaces::srv::SpawnEntity::Request>();
  request->name = "test_model";
  request->entity_namespace = expected_namespace;
  request->entity_resource.resource_string =
    "<sdf version='1.12'><model name='test_model'>"
    "<link name='link'/></model></sdf>";

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(ros_node, future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->result.result,
    simulation_interfaces::msg::Result::RESULT_OK);

  server->Stop();

  {
    auto ecm = server->EcmScope();
    ASSERT_TRUE(ecm.Valid());

    auto modelEntity = server->EntityByName("test_model");
    ASSERT_TRUE(modelEntity.has_value());

    auto ns_comp = ecm->Component<gz::sim::components::Namespace>(*modelEntity);
    ASSERT_TRUE(ns_comp);
    EXPECT_EQ(ns_comp->Data(), expected_namespace);
  }

  server.reset();
  sim_interfaces.reset();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
