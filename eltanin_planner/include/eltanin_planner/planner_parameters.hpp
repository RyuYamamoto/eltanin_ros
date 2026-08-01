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
#include <eltanin/planner/hybrid_astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin_ros_common/conversion_result.hpp>

#include <cstddef>
#include <optional>
#include <string_view>

namespace eltanin_planner
{

inline constexpr const char * KEY_PLANNER_TYPE = "planner_type";
inline constexpr const char * KEY_START_SEARCH_RADIUS_CELLS = "start_search_radius_cells";
inline constexpr const char * KEY_WEIGHT_DATA = "weight_data";
inline constexpr const char * KEY_WEIGHT_SMOOTH = "weight_smooth";
inline constexpr const char * KEY_SMOOTHER_TOLERANCE = "smoother_tolerance";
inline constexpr const char * KEY_SMOOTHER_MAX_ITERATIONS = "smoother_max_iterations";
inline constexpr const char * KEY_PUBLISH_RAW_PATH = "publish_raw_path";
inline constexpr const char * KEY_UNKNOWN_IS_FREE = "unknown_is_free";
inline constexpr const char * KEY_TF_LOOKUP_TIMEOUT = "tf_lookup_timeout";
inline constexpr const char * KEY_PUBLISH_FOOTPRINT_PATH = "publish_footprint_path";
inline constexpr const char * KEY_FOOTPRINT_MARKER_STRIDE = "footprint_marker_stride";
inline constexpr const char * KEY_HEADING_BINS = "hybrid.heading_bins";
inline constexpr const char * KEY_MINIMUM_TURNING_RADIUS = "hybrid.minimum_turning_radius";
inline constexpr const char * KEY_MOTION_STEP = "hybrid.motion_step";
inline constexpr const char * KEY_COLLISION_CHECK_STEP = "hybrid.collision_check_step";
inline constexpr const char * KEY_DUBINS_EXPANSION_DISTANCE = "hybrid.dubins_expansion_distance";
inline constexpr const char * KEY_STEERING_PENALTY = "hybrid.steering_penalty";
inline constexpr const char * KEY_STEERING_CHANGE_PENALTY = "hybrid.steering_change_penalty";
inline constexpr const char * KEY_MAX_EXPANSIONS = "hybrid.max_expansions";
inline constexpr const char * KEY_MAX_STATES = "hybrid.max_states";

/// Which search runs. Both go through the same Planner::plan(), so the failure classes are shared.
enum class PlannerType { AStar, HybridAStar };

/// The parameter spelling of a PlannerType.
const char * name_of(PlannerType type) noexcept;

/// The inverse; nullopt for a name nobody defined.
std::optional<PlannerType> to_planner_type(std::string_view name) noexcept;

/// eltanin's own parameter structs are held by value, so the two sets of defaults cannot drift.
struct PlannerParameters
{
  PlannerType planner_type{PlannerType::AStar};
  eltanin::planner::AStarParams astar{};
  eltanin::planner::HybridAStarParams hybrid{};
  eltanin::planner::SmootherParams smoother{};
  bool publish_raw_path{false};
  bool unknown_is_free{false};
  double tf_lookup_timeout{0.1};
  bool publish_footprint_path{false};
  int footprint_marker_stride{10};
  /// Ceiling on cells * heading_bins; eltanin allocates the whole state space before searching.
  std::size_t hybrid_max_states{20000000};
};

/// The first violated condition only; both searches are checked whichever one is selected.
eltanin_ros_common::ConversionStatus validate(const PlannerParameters & parameters);

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__PLANNER_PARAMETERS_HPP_
