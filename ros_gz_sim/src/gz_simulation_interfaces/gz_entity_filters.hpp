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

#ifndef GZ_SIMULATION_INTERFACES__GZ_ENTITY_FILTERS_HPP_
#define GZ_SIMULATION_INTERFACES__GZ_ENTITY_FILTERS_HPP_

#include <regex>
#include <string>
#include <tuple>

#include <gz/sim/Server.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SemanticCategory.hh>
#include <gz/sim/components/SemanticTags.hh>

#include "simulation_interfaces/msg/entity_filters.hpp"
#include "simulation_interfaces/msg/result.hpp"
namespace ros_gz_sim
{
namespace gz_simulation_interfaces
{

/// \class GzEntityFilters
/// \brief Implements the various ways in which entities can be filtered in services such as
/// `GetEntities`.
class GzEntityFilters
{
public:
  /// \brief Constructor.
  /// \param[in] filters Filters to apply.
  /// \param[in] ecm Reference to the EntityComponentManager.
  GzEntityFilters(
    const simulation_interfaces::msg::EntityFilters & filters,
    const gz::sim::EntityComponentManager & ecm);

  /// \brief Checks whether the given entity matches the filter.
  /// \param[in] entity ID of the entity in the EntityComponentManager.
  /// \param[in] entity_name Name of the entity.
  ///
  /// \note The fact that both `entity` and `entity_name` are needed is an optimization. Having both
  /// avoid duplicate calls to fetch the ID or name.
  std::tuple<bool, simulation_interfaces::msg::Result> ApplyFilter(
    const gz::sim::Entity & entity, const std::string entity_name);

private:
  /// \brief Filters to apply.
  simulation_interfaces::msg::EntityFilters filters_;
  /// \brief Reference to the EntityComponentManager.
  const gz::sim::EntityComponentManager & ecm_;
  /// \brief Regular expression object created from the contents of `filters_`.
  std::regex regex_filter_;
};
}  // namespace gz_simulation_interfaces
}  // namespace ros_gz_sim
#endif  // GZ_SIMULATION_INTERFACES__GZ_ENTITY_FILTERS_HPP_
