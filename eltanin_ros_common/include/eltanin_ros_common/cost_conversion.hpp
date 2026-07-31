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

#ifndef ELTANIN_ROS_COMMON__COST_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__COST_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <cstdint>
#include <vector>

namespace eltanin_ros_common
{

/// Thresholds for reducing an OccupancyGrid to the three values a static map really carries.
struct OccupancyThresholds
{
  int occupied_threshold{65};
  int free_threshold{25};
};

/// Requires 0 <= free_threshold < occupied_threshold <= 100.
ConversionStatus validate(const OccupancyThresholds & thresholds);

/// One cell of a static map; negative and in-between values are both unknown.
std::uint8_t occupancy_to_cost(
  std::int8_t occupancy, const OccupancyThresholds & thresholds) noexcept;

/// One cell for visualization only; 252 cost steps collapse into 98, so this does not round-trip.
std::int8_t cost_to_occupancy(std::uint8_t cost) noexcept;

/// Bulk form of occupancy_to_cost; `out` is resized and overwritten, never appended to.
void occupancy_to_costs(
  const std::vector<std::int8_t> & occupancy, const OccupancyThresholds & thresholds,
  std::vector<std::uint8_t> & out);

/// Bulk form of cost_to_occupancy; `out` is resized and overwritten, never appended to.
void costs_to_occupancy(const std::vector<std::uint8_t> & costs, std::vector<std::int8_t> & out);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__COST_CONVERSION_HPP_
