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
inline constexpr const char * KEY_ANALYTIC_EXPANSION_RATIO = "hybrid.analytic_expansion_ratio";
inline constexpr const char * KEY_MOTION_MODEL = "hybrid.motion_model";
inline constexpr const char * KEY_HEURISTIC_WEIGHT = "hybrid.heuristic_weight";
inline constexpr const char * KEY_MAX_STATES = "hybrid.max_states";
inline constexpr const char * KEY_CORRIDOR_MARGIN_CELLS = "hybrid.corridor_margin_cells";

/// Which search runs. Both go through the same Planner::plan(), so the failure classes are shared.
enum class PlannerType { AStar, HybridAStar };

/// The parameter spelling of a PlannerType.
const char * name_of(PlannerType type) noexcept;

/// The inverse; nullopt for a name nobody defined.
std::optional<PlannerType> to_planner_type(std::string_view name) noexcept;

/// The parameter spelling of the control set the vehicle is allowed to use.
const char * name_of(eltanin::planner::MotionModel model) noexcept;

/// The inverse; nullopt for a name nobody defined.
std::optional<eltanin::planner::MotionModel> to_motion_model(std::string_view name) noexcept;

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
  /// Ceiling on cells * heading_bins. eltanin refuses oversized problems itself now, but this
  /// keeps the rejection on the ROS side where it can name the map.
  std::size_t hybrid_max_states{20000000};
  /// Half width of the corridor Hybrid A* searches around the raw A* guide [cells].
  int hybrid_corridor_margin_cells{30};
};

/// The first violated condition only; both searches are checked whichever one is selected.
eltanin_ros_common::ConversionStatus validate(const PlannerParameters & parameters);

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__PLANNER_PARAMETERS_HPP_
