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

#ifndef ELTANIN_PLANNER__PLANNER_PARAMETERS_HPP_
#define ELTANIN_PLANNER__PLANNER_PARAMETERS_HPP_

#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin_ros_common/conversion_result.hpp>

namespace eltanin_planner
{

inline constexpr const char * KEY_START_SEARCH_RADIUS_CELLS = "start_search_radius_cells";
inline constexpr const char * KEY_WEIGHT_DATA = "weight_data";
inline constexpr const char * KEY_WEIGHT_SMOOTH = "weight_smooth";
inline constexpr const char * KEY_SMOOTHER_TOLERANCE = "smoother_tolerance";
inline constexpr const char * KEY_SMOOTHER_MAX_ITERATIONS = "smoother_max_iterations";
inline constexpr const char * KEY_PUBLISH_RAW_PATH = "publish_raw_path";
inline constexpr const char * KEY_UNKNOWN_IS_FREE = "unknown_is_free";
inline constexpr const char * KEY_TF_LOOKUP_TIMEOUT = "tf_lookup_timeout";

/// eltanin's own parameter structs are held by value, so the two sets of defaults cannot drift.
struct PlannerParameters
{
  eltanin::planner::AStarParams astar{};
  eltanin::planner::SmootherParams smoother{};
  bool publish_raw_path{false};
  bool unknown_is_free{false};
  double tf_lookup_timeout{0.1};
};

/// The first violated condition only; these are the preconditions eltanin states with assert alone.
eltanin_ros_common::ConversionStatus validate(const PlannerParameters & parameters);

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__PLANNER_PARAMETERS_HPP_
