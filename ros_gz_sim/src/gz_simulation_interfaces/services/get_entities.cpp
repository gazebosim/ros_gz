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

#include "get_entities.hpp"

#include <gz/msgs/boolean.pb.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <regex>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SemanticCategory.hh>
#include <gz/sim/components/SemanticTags.hh>

#include "../gazebo_proxy.hpp"
#include "simulation_interfaces/msg/entity_category.hpp"
#include "simulation_interfaces/msg/result.hpp"
#include "simulation_interfaces/msg/tags_filter.hpp"
#include "simulation_interfaces/srv/get_entities.hpp"

namespace components = gz::sim::components;

namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
namespace services
{
using GetEntitiesSrv = simulation_interfaces::srv::GetEntities;
using RequestPtr = GetEntitiesSrv::Request::ConstSharedPtr;
using ResponsePtr = GetEntitiesSrv::Response::SharedPtr;
using simulation_interfaces::msg::EntityCategory;
using simulation_interfaces::msg::Result;
using simulation_interfaces::msg::TagsFilter;

GetEntities::GetEntities(
  std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<GazeboProxy> gz_proxy)
: HandlerBase(ros_node, gz_proxy)
{
  auto service_cb = [this](RequestPtr request, ResponsePtr response) {
    std::regex regex_filter;
    try {
      regex_filter = std::regex(request->filters.filter, std::regex::extended);
    } catch (const std::regex_error & e) {
      response->result.result = Result::RESULT_OPERATION_FAILED;
      response->result.error_message = e.what();
      return;
    }

    const auto & tags_filter_mode = request->filters.tags.filter_mode;
    if (
      tags_filter_mode != TagsFilter::FILTER_MODE_ANY &&
      tags_filter_mode != TagsFilter::FILTER_MODE_ALL) {
      response->result.result = Result::RESULT_OPERATION_FAILED;
      response->result.error_message =
        "The tag filter mode needs to be one of [FILTER_MODE_ANY, FILTER_MODE_ALL]";
      return;
    }

    this->gz_proxy_->WithEcm([&](const gz::sim::EntityComponentManager & ecm) {
      ecm.Each<components::Name, components::Model, components::ParentEntity>(
        [&](
          const gz::sim::Entity & entity, const components::Name * name, const components::Model *,
          const components::ParentEntity * parent) {
          // Check that this is a top level model
          if (ecm.Component<components::Model>(parent->Data())) {
            // This is a nested model which should not be included in the list of entities to
            // return.
            // TODO(azeey) It might be useful to allow nested models here when we enable setting
            // their poses in Gazebo.
            return true;
          }

          if (!request->filters.filter.empty() && !std::regex_match(name->Data(), regex_filter)) {
            return true;
          }
          const std::vector<EntityCategory> & test_categories = request->filters.categories;
          if (!test_categories.empty()) {
            EntityCategory entity_category;
            entity_category.category =
              ecm.ComponentData<components::SemanticCategory>(entity).value_or(
                EntityCategory::CATEGORY_OBJECT);
            if (
              std::find(test_categories.begin(), test_categories.end(), entity_category) ==
              test_categories.end()) {
              return true;
            }
          }
          const auto & test_tags = request->filters.tags.tags;
          if (!test_tags.empty()) {
            const auto entity_tags = ecm.ComponentData<components::SemanticTags>(entity).value_or(
              components::SemanticTags::Type{});
            auto are_in_entity_tags = [&entity_tags](auto tag) {
              return std::find(entity_tags.begin(), entity_tags.end(), tag) != entity_tags.end();
            };

            if (request->filters.tags.filter_mode == TagsFilter::FILTER_MODE_ANY) {
              if (!std::any_of(test_tags.begin(), test_tags.end(), are_in_entity_tags)) {
                return true;
              }
            } else if (request->filters.tags.filter_mode == TagsFilter::FILTER_MODE_ALL) {
              if (!std::all_of(test_tags.begin(), test_tags.end(), are_in_entity_tags)) {
                return true;
              }
            }
          }

          response->entities.push_back(name->Data());
          return true;
        });

      // TODO(azeey) Implement filtering by bounds
    });
  };

  this->services_handle_ = ros_node->create_service<GetEntitiesSrv>("get_entities", service_cb);

  RCLCPP_INFO_STREAM(ros_node->get_logger(), "Created service " << "get_entities");
}
}  // namespace services
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
