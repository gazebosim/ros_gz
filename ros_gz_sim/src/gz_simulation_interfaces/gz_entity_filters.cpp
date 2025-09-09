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

#include "gz_entity_filters.hpp"

#include <algorithm>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <gz/sim/Server.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SemanticCategory.hh>
#include <gz/sim/components/SemanticTags.hh>

#include "simulation_interfaces/msg/entity_category.hpp"
#include "simulation_interfaces/msg/entity_filters.hpp"
#include "simulation_interfaces/msg/result.hpp"
#include "simulation_interfaces/msg/tags_filter.hpp"
namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{
using simulation_interfaces::msg::EntityCategory;
using simulation_interfaces::msg::Result;
using simulation_interfaces::msg::TagsFilter;

GzEntityFilters::GzEntityFilters(
  const simulation_interfaces::msg::EntityFilters & filters,
  const gz::sim::EntityComponentManager & ecm)
: filters_(filters), ecm_(ecm), regex_filter_(filters.filter, std::regex::extended)
{
  const auto & tags_filter_mode = filters_.tags.filter_mode;
  if (tags_filter_mode != TagsFilter::FILTER_MODE_ANY &&
    tags_filter_mode != TagsFilter::FILTER_MODE_ALL)
  {
    throw std::runtime_error(
      "The tag filter mode needs to be one of [FILTER_MODE_ANY, FILTER_MODE_ALL]");
  }
}

std::tuple<bool, Result> GzEntityFilters::ApplyFilter(
  const gz::sim::Entity & entity, const std::string entity_name)
{
  namespace components = gz::sim::components;
  Result result;

  result.result = Result::RESULT_OK;
  if (!filters_.filter.empty() && !std::regex_match(entity_name, regex_filter_)) {
    return {false, result};
  }

  const std::vector<EntityCategory> & test_categories = filters_.categories;
  if (!test_categories.empty()) {
    EntityCategory entity_category;
    entity_category.category = ecm_.ComponentData<components::SemanticCategory>(entity).value_or(
      EntityCategory::CATEGORY_OBJECT);
    auto it = std::find(test_categories.begin(), test_categories.end(), entity_category);
    if (it == test_categories.end()) {
      return {false, result};
    }
  }
  const auto & test_tags = filters_.tags.tags;
  if (!test_tags.empty()) {
    const auto entity_tags = ecm_.ComponentData<components::SemanticTags>(entity).value_or(
      components::SemanticTags::Type{});
    auto are_in_entity_tags =
      [&entity_tags](auto tag) {
        return std::find(entity_tags.begin(), entity_tags.end(), tag) != entity_tags.end();
      };

    if (filters_.tags.filter_mode == TagsFilter::FILTER_MODE_ANY) {
      if (!std::any_of(test_tags.begin(), test_tags.end(), are_in_entity_tags)) {
        return {false, result};
      }
    } else if (filters_.tags.filter_mode == TagsFilter::FILTER_MODE_ALL) {
      if (!std::all_of(test_tags.begin(), test_tags.end(), are_in_entity_tags)) {
        return {false, result};
      }
    }
  }
  // TODO(azeey) Implement filtering by bounds
  return {true, result};
}
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
