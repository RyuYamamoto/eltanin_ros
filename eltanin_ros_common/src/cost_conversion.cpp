// Copyright 2026 RyuYamamoto.
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
// limitations under the License

#include "eltanin_ros_common/cost_conversion.hpp"

#include "src/diagnostic.hpp"

#include <eltanin/map/cost_values.hpp>

#include <cassert>
#include <string>

namespace eltanin_ros_common
{

namespace
{

constexpr int OCCUPANCY_MAX = 100;

/// Highest cost that carries an inflation value rather than being a reserved sentinel.
constexpr int VISUALIZED_COST_MAX = eltanin::map::MAX_NON_OBSTACLE;

/// Span of the visualized range 1..98, matching costmap_2d's cost_translation_table_.
constexpr int VISUALIZED_SPAN = 97;

constexpr int VISUALIZED_DIVISOR = VISUALIZED_COST_MAX - 1;

}  // namespace

ConversionStatus validate(const OccupancyThresholds & thresholds)
{
  if (thresholds.free_threshold < 0) {
    return ConversionStatus::failure(diagnostic::rejected(
      "occupancy thresholds",
      "free_threshold is " + std::to_string(thresholds.free_threshold) + ", must be at least 0"));
  }
  if (thresholds.occupied_threshold > OCCUPANCY_MAX) {
    return ConversionStatus::failure(diagnostic::rejected(
      "occupancy thresholds", "occupied_threshold is " +
                                std::to_string(thresholds.occupied_threshold) +
                                ", must be at most " + std::to_string(OCCUPANCY_MAX)));
  }
  if (thresholds.free_threshold >= thresholds.occupied_threshold) {
    return ConversionStatus::failure(diagnostic::rejected(
      "occupancy thresholds", "free_threshold " + std::to_string(thresholds.free_threshold) +
                                " must be below occupied_threshold " +
                                std::to_string(thresholds.occupied_threshold)));
  }
  return ConversionStatus::success();
}

std::uint8_t occupancy_to_cost(
  std::int8_t occupancy, const OccupancyThresholds & thresholds) noexcept
{
  const int value = occupancy;
  if (value < 0) {
    return eltanin::map::NO_INFORMATION;
  }
  if (value >= thresholds.occupied_threshold) {
    return eltanin::map::LETHAL_OBSTACLE;
  }
  if (value <= thresholds.free_threshold) {
    return eltanin::map::FREE_SPACE;
  }
  return eltanin::map::NO_INFORMATION;
}

std::int8_t cost_to_occupancy(std::uint8_t cost) noexcept
{
  if (cost == eltanin::map::NO_INFORMATION) {
    return -1;
  }
  if (cost == eltanin::map::LETHAL_OBSTACLE) {
    return 100;
  }
  if (cost == eltanin::map::INSCRIBED_INFLATED_OBSTACLE) {
    return 99;
  }
  if (cost == eltanin::map::FREE_SPACE) {
    return 0;
  }
  const int value = cost;
  return static_cast<std::int8_t>(1 + (VISUALIZED_SPAN * (value - 1)) / VISUALIZED_DIVISOR);
}

void occupancy_to_costs(
  const std::vector<std::int8_t> & occupancy, const OccupancyThresholds & thresholds,
  std::vector<std::uint8_t> & out)
{
  assert(validate(thresholds).ok());
  out.resize(occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    out[i] = occupancy_to_cost(occupancy[i], thresholds);
  }
}

void costs_to_occupancy(const std::vector<std::uint8_t> & costs, std::vector<std::int8_t> & out)
{
  out.resize(costs.size());
  for (std::size_t i = 0; i < costs.size(); ++i) {
    out[i] = cost_to_occupancy(costs[i]);
  }
}

}  // namespace eltanin_ros_common
