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

#include "spawn_entity.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Actor.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/srv/spawn_entity.hpp"

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using SpawnEntitySrv = simulation_interfaces::srv::SpawnEntity;
using RequestPtr = SpawnEntitySrv::Request::ConstSharedPtr;
using ResponsePtr = SpawnEntitySrv::Response::SharedPtr;
namespace components = gz::sim::components;

SpawnEntity::SpawnEntity(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  const auto create_service = this->gz_proxy_->PrefixTopic("create/blocking");
  if (!this->gz_proxy_->WaitForGzService(create_service)) {
    RCLCPP_ERROR_STREAM(
      this->ros_node_->get_logger(),
      "Gazebo service [" << create_service << "] is not available. "
                         << "The [SpawnEntity] interface will not function properly. To fix this, "
                            "make sure the [UserCommands] system is loaded in your Gazebo world");
  }
  this->services_handle_ = ros_node->create_service<SpawnEntitySrv>(
    "spawn_entity", [this, create_service](RequestPtr request, ResponsePtr response) {
      using Result = simulation_interfaces::msg::Result;
      gz::msgs::EntityFactory gz_request;
      if (!request->name.empty()) {
        gz_request.set_name(request->name);
      }
      gz_request.set_allow_renaming(request->allow_renaming);
      const auto & resource = request->entity_resource;

      if (!resource.uri.empty()) {
        // TODO(azeey) The `sdf_filename` field requires absolute paths to the file.
        // Consider resolving the uri using the `/gazebo/resource_paths/resolve`
        gz_request.set_sdf_filename(resource.uri);
      } else if (!resource.resource_string.empty()) {
        gz_request.set_sdf(resource.resource_string);
      } else {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message =
        "One of the fields [uri] or [resource_string] must be specified";
        return;
      }

      std::string expected_name = request->name;
      if (expected_name.empty()) {
        sdf::Root root;
        if (!resource.uri.empty()) {
          root.Load(resource.uri);
        } else {
          root.LoadSdfString(resource.resource_string);
        }
        if (root.Model()) {
          expected_name = root.Model()->Name();
        } else if (root.Light()) {
          expected_name = root.Light()->Name();
        } else if (root.Actor()) {
          expected_name = root.Actor()->Name();
        }
      }

      if (!this->gz_proxy_->AssertUpdatedState(response->result)) {
        return;
      }

      std::unordered_set<gz::sim::Entity> entities_before;
      bool expected_name_exists = false;
      this->gz_proxy_->WithEcm([&](const gz::sim::EntityComponentManager & ecm) {
        ecm.Each<components::Name, components::ParentEntity>(
          [&](const gz::sim::Entity & entity, const components::Name * name,
          const components::ParentEntity * parent) {
            if (ecm.Component<components::World>(parent->Data())) {
              entities_before.insert(entity);
              expected_name_exists |= name->Data() == expected_name;
            }
            return true;
          });
      });
      if (!expected_name.empty() && expected_name_exists && !request->allow_renaming) {
        response->result.result = SpawnEntitySrv::Response::NAME_NOT_UNIQUE;
        response->result.error_message = "An entity with the requested name already exists";
        return;
      }

      // TODO(azeey) Add support for entity_namespace
      // TODO(azeey) Reuse code in ros_gz_bridge/convert/geometry_msgs
      auto * pose = gz_request.mutable_pose();
      pose->mutable_position()->set_x(request->initial_pose.pose.position.x);
      pose->mutable_position()->set_y(request->initial_pose.pose.position.y);
      pose->mutable_position()->set_z(request->initial_pose.pose.position.z);

      pose->mutable_orientation()->set_x(request->initial_pose.pose.orientation.x);
      pose->mutable_orientation()->set_y(request->initial_pose.pose.orientation.y);
      pose->mutable_orientation()->set_z(request->initial_pose.pose.orientation.z);
      pose->mutable_orientation()->set_w(request->initial_pose.pose.orientation.w);

      bool result;
      gz::msgs::Boolean reply;
      bool executed = this->gz_proxy_->GzNode()->Request(
        create_service, gz_request, GazeboProxy::kGzServiceTimeoutMs, reply, result);
      if (!executed) {
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Timed out while trying to spawn entity";
      } else if (result && reply.data()) {
        const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(GazeboProxy::kGzStateUpdatedTimeoutMs);
        while (true) {
          std::vector<std::string> matching_names;
          this->gz_proxy_->WithEcm([&](const gz::sim::EntityComponentManager & ecm) {
            ecm.Each<components::Name, components::ParentEntity>(
              [&](const gz::sim::Entity & entity, const components::Name * name,
              const components::ParentEntity * parent) {
                const bool name_matches = expected_name.empty() ||
                (request->allow_renaming ?
                name->Data().compare(0, expected_name.size(), expected_name) == 0 :
                name->Data() == expected_name);
                if (
                  ecm.Component<components::World>(parent->Data()) &&
                  entities_before.count(entity) == 0 && name_matches)
                {
                  matching_names.push_back(name->Data());
                }
                return true;
              });
          });
          if (matching_names.size() == 1) {
            response->entity_name = matching_names.front();
            response->result.result = Result::RESULT_OK;
            return;
          }
          if (matching_names.size() > 1 || std::chrono::steady_clock::now() >= deadline) {
            response->result.result = Result::RESULT_OPERATION_FAILED;
            response->result.error_message = "Unable to identify the newly spawned entity";
            return;
          }
          simulation_interfaces::msg::Result state_result;
          if (!this->gz_proxy_->AssertUpdatedState(state_result)) {
            response->result = state_result;
            return;
          }
        }
      } else {
        // TODO(azeey) SpawnEntity has additional error codes to allow surfacing more informative
        // error messages. However, the `create` service in UserCommands only returns a boolean.
        response->result.result = Result::RESULT_OPERATION_FAILED;
        response->result.error_message = "Unknown error while trying to spawn entity";
      }
    });

  RCLCPP_INFO_STREAM(
    ros_node->get_logger(), "Created service " << this->services_handle_->get_service_name());
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
