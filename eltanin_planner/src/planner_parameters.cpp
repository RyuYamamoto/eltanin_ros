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

#include "eltanin_planner/planner_parameters.hpp"

#include "src/diagnostic.hpp"

#include <cmath>
#include <string>

namespace eltanin_planner
{

namespace
{

using eltanin_ros_common::ConversionStatus;

/// The bound assert_smoother_params() states; it lives in a .cpp, so RelWithDebInfo drops it.
constexpr double SMOOTHER_CONVERGENCE_BOUND = 2.0;

ConversionStatus require_finite_non_negative(const char * key, double value)
{
  if (!std::isfinite(value)) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be finite"));
  }
  if (value < 0.0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must not be negative"));
  }
  return ConversionStatus::success();
}

ConversionStatus require_finite_positive(const char * key, double value)
{
  if (!std::isfinite(value)) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be finite"));
  }
  if (value <= 0.0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be greater than 0"));
  }
  return ConversionStatus::success();
}

ConversionStatus require_at_least(const char * key, int value, int lowest)
{
  if (value < lowest) {
    return ConversionStatus::failure(diagnostic::rejected(
      key, "is " + std::to_string(value) + ", which must be at least " + std::to_string(lowest)));
  }
  return ConversionStatus::success();
}

ConversionStatus require_non_negative(const char * key, int value)
{
  return require_at_least(key, value, 0);
}

/// The conditions HybridAStarPlanner's constructor throws on, checked where they can be reported.
ConversionStatus validate_hybrid(const eltanin::planner::HybridAStarParams & hybrid)
{
  const ConversionStatus radius_cells =
    require_non_negative(KEY_START_SEARCH_RADIUS_CELLS, hybrid.common.start_search_radius_cells);
  if (!radius_cells.ok()) {
    return radius_cells;
  }
  const ConversionStatus bins = require_at_least(KEY_HEADING_BINS, hybrid.heading_bins, 8);
  if (!bins.ok()) {
    return bins;
  }
  const ConversionStatus radius =
    require_finite_positive(KEY_MINIMUM_TURNING_RADIUS, hybrid.minimum_turning_radius);
  if (!radius.ok()) {
    return radius;
  }
  const ConversionStatus motion = require_finite_non_negative(KEY_MOTION_STEP, hybrid.motion_step);
  if (!motion.ok()) {
    return motion;
  }
  const ConversionStatus check =
    require_finite_non_negative(KEY_COLLISION_CHECK_STEP, hybrid.collision_check_step);
  if (!check.ok()) {
    return check;
  }
  const ConversionStatus dubins =
    require_finite_positive(KEY_DUBINS_EXPANSION_DISTANCE, hybrid.dubins_expansion_distance);
  if (!dubins.ok()) {
    return dubins;
  }
  const ConversionStatus steering =
    require_finite_non_negative(KEY_STEERING_PENALTY, hybrid.steering_penalty);
  if (!steering.ok()) {
    return steering;
  }
  const ConversionStatus change =
    require_finite_non_negative(KEY_STEERING_CHANGE_PENALTY, hybrid.steering_change_penalty);
  if (!change.ok()) {
    return change;
  }
  const ConversionStatus ratio =
    require_finite_positive(KEY_ANALYTIC_EXPANSION_RATIO, hybrid.analytic_expansion_ratio);
  if (!ratio.ok()) {
    return ratio;
  }
  return require_finite_non_negative(KEY_HEURISTIC_WEIGHT, hybrid.heuristic_weight);
}

}  // namespace

const char * name_of(eltanin::planner::MotionModel model) noexcept
{
  return model == eltanin::planner::MotionModel::Differential ? "differential" : "dubins";
}

std::optional<eltanin::planner::MotionModel> to_motion_model(std::string_view name) noexcept
{
  if (name == "dubins") {
    return eltanin::planner::MotionModel::Dubins;
  }
  if (name == "differential") {
    return eltanin::planner::MotionModel::Differential;
  }
  return std::nullopt;
}

const char * name_of(PlannerType type) noexcept
{
  return type == PlannerType::HybridAStar ? "hybrid_astar" : "astar";
}

std::optional<PlannerType> to_planner_type(std::string_view name) noexcept
{
  if (name == "astar") {
    return PlannerType::AStar;
  }
  if (name == "hybrid_astar") {
    return PlannerType::HybridAStar;
  }
  return std::nullopt;
}

ConversionStatus validate(const PlannerParameters & parameters)
{
  const ConversionStatus radius = require_non_negative(
    KEY_START_SEARCH_RADIUS_CELLS, parameters.astar.common.start_search_radius_cells);
  if (!radius.ok()) {
    return radius;
  }
  const ConversionStatus weight_data =
    require_finite_non_negative(KEY_WEIGHT_DATA, parameters.smoother.weight_data);
  if (!weight_data.ok()) {
    return weight_data;
  }
  const ConversionStatus weight_smooth =
    require_finite_non_negative(KEY_WEIGHT_SMOOTH, parameters.smoother.weight_smooth);
  if (!weight_smooth.ok()) {
    return weight_smooth;
  }
  const ConversionStatus tolerance =
    require_finite_non_negative(KEY_SMOOTHER_TOLERANCE, parameters.smoother.tolerance);
  if (!tolerance.ok()) {
    return tolerance;
  }
  const ConversionStatus iterations =
    require_non_negative(KEY_SMOOTHER_MAX_ITERATIONS, parameters.smoother.max_iterations);
  if (!iterations.ok()) {
    return iterations;
  }
  const ConversionStatus timeout =
    require_finite_non_negative(KEY_TF_LOOKUP_TIMEOUT, parameters.tf_lookup_timeout);
  if (!timeout.ok()) {
    return timeout;
  }

  const double bound = parameters.smoother.weight_data + 4.0 * parameters.smoother.weight_smooth;
  if (bound >= SMOOTHER_CONVERGENCE_BOUND) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_WEIGHT_SMOOTH,
      "weight_data " + std::to_string(parameters.smoother.weight_data) + " + 4 * weight_smooth " +
        std::to_string(parameters.smoother.weight_smooth) + " is " + std::to_string(bound) +
        ", which must be below " + std::to_string(SMOOTHER_CONVERGENCE_BOUND) +
        " or the smoother diverges"));
  }

  const ConversionStatus stride =
    require_at_least(KEY_FOOTPRINT_MARKER_STRIDE, parameters.footprint_marker_stride, 1);
  if (!stride.ok()) {
    return stride;
  }
  const ConversionStatus margin =
    require_non_negative(KEY_CORRIDOR_MARGIN_CELLS, parameters.hybrid_corridor_margin_cells);
  if (!margin.ok()) {
    return margin;
  }
  if (parameters.hybrid_max_states == 0) {
    return ConversionStatus::failure(
      diagnostic::rejected(KEY_MAX_STATES, "is 0, which allows no state at all"));
  }
  return validate_hybrid(parameters.hybrid);
}

}  // namespace eltanin_planner
