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

#include "eltanin_planner/plan_attempt.hpp"

#include "src/diagnostic.hpp"

#include <eltanin/map/crop.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <eltanin_msgs/msg/navigation_state.hpp>

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace eltanin_planner
{

namespace
{

using eltanin::Traversability;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;

const char * traversability_name(Traversability value) noexcept
{
  switch (value) {
    case Traversability::Free:
      return "Free";
    case Traversability::Circumscribed:
      return "Circumscribed";
    case Traversability::Inscribed:
      return "Inscribed";
  }
  return "an unnamed class";
}

/// Every rejection names the map it was judged against, so the one line stands on its own.
std::string describe_map(const MapGeometry & geometry)
{
  return std::to_string(geometry.size_x()) + "x" + std::to_string(geometry.size_y()) + " map at " +
         std::to_string(geometry.resolution()) + " m from (" +
         std::to_string(geometry.origin().x()) + ", " + std::to_string(geometry.origin().y()) + ")";
}

std::string describe_point(const char * what, const Eigen::Vector2d & position)
{
  return std::string(what) + " (" + std::to_string(position.x()) + ", " +
         std::to_string(position.y()) + ")";
}

std::string describe_cell(const char * what, const MapIndex & index)
{
  return std::string(what) + " cell (" + std::to_string(index.x) + ", " + std::to_string(index.y) +
         ")";
}

/// eltanin classifies the search failure itself now; only the ones it can reach are mapped.
PlanFailure failure_of(eltanin::planner::PlannerError error) noexcept
{
  switch (error) {
    case eltanin::planner::PlannerError::StateSpaceTooLarge:
      return PlanFailure::StateSpaceTooLarge;
    case eltanin::planner::PlannerError::StartOutsideMap:
      return PlanFailure::StartOutsideMap;
    case eltanin::planner::PlannerError::GoalOutsideMap:
      return PlanFailure::GoalOutsideMap;
    case eltanin::planner::PlannerError::GoalBlocked:
      return PlanFailure::GoalNotFree;
    case eltanin::planner::PlannerError::StartRescueFailed:
      return PlanFailure::StartNotRescuable;
    case eltanin::planner::PlannerError::InvalidMap:
      return PlanFailure::MapUnusable;
    default:
      return PlanFailure::SearchFailed;
  }
}

PlanAttempt fail(PlanFailure failure, std::string message)
{
  return PlanAttempt{eltanin::Path{}, failure, std::move(message)};
}

/// Both searches rescue the start themselves, and the pre-check has to use the same radius.
int search_radius(const PlannerParameters & parameters) noexcept
{
  return parameters.planner_type == PlannerType::HybridAStar
           ? parameters.hybrid.common.start_search_radius_cells
           : parameters.astar.common.start_search_radius_cells;
}

}  // namespace

std::uint8_t to_outcome(PlanFailure failure) noexcept
{
  using eltanin_msgs::msg::NavigationState;
  switch (failure) {
    case PlanFailure::None:
      return NavigationState::OUTCOME_REACHED;
    case PlanFailure::MapUnusable:
      return NavigationState::OUTCOME_INPUT_STALE;
    case PlanFailure::StartOutsideMap:
    case PlanFailure::GoalOutsideMap:
    case PlanFailure::StartNotRescuable:
      return NavigationState::OUTCOME_START_GOAL_FAILED;
    case PlanFailure::StateSpaceTooLarge:
      return NavigationState::OUTCOME_PLAN_FAILED;
    case PlanFailure::GoalNotFree:
      return NavigationState::OUTCOME_NO_PATH;
    case PlanFailure::SearchFailed:
    case PlanFailure::EmptyPath:
      return NavigationState::OUTCOME_PLAN_FAILED;
  }
  return NavigationState::OUTCOME_UNKNOWN;
}

PlanAttempt attempt_plan(
  const eltanin::map::Costmap & costmap, const eltanin::map::CostTraversabilityModel & model,
  const eltanin::Pose2D & start, const eltanin::Pose2D & goal, const PlannerParameters & parameters)
{
  // Deliberate copy of eltanin plan() (planner.hpp:83-104); same order/radius/model (D-T7-5).
  const MapGeometry & geometry = costmap.geometry();
  if (!std::isfinite(geometry.resolution()) || geometry.resolution() <= 0.0) {
    return fail(
      PlanFailure::MapUnusable,
      diagnostic::rejected("the costmap", "resolution must be finite and greater than 0"));
  }
  if (costmap.cell_count() == 0) {
    return fail(PlanFailure::MapUnusable, diagnostic::rejected("the costmap", "it has no cells"));
  }

  const std::optional<MapIndex> start_cell = geometry.world_to_map(start.position);
  const std::optional<MapIndex> goal_cell = geometry.world_to_map(goal.position);
  if (!start_cell.has_value()) {
    return fail(
      PlanFailure::StartOutsideMap,
      diagnostic::rejected(
        describe_point("start", start.position), "it lies outside the " + describe_map(geometry)));
  }
  if (!goal_cell.has_value()) {
    return fail(
      PlanFailure::GoalOutsideMap,
      diagnostic::rejected(
        describe_point("goal", goal.position), "it lies outside the " + describe_map(geometry)));
  }

  const std::uint8_t goal_cost = costmap.get(goal_cell->x, goal_cell->y).value();
  const Traversability goal_class = model.classify(goal_cost);
  if (goal_class != Traversability::Free) {
    return fail(
      PlanFailure::GoalNotFree,
      diagnostic::rejected(
        describe_cell("goal", *goal_cell), "cost " + std::to_string(goal_cost) + " classifies as " +
                                             traversability_name(goal_class) +
                                             ", not Free; the goal is reported, not moved"));
  }

  const int radius = search_radius(parameters);
  const std::optional<MapIndex> rescued =
    eltanin::planner::find_nearest_traversable(costmap, model, *start_cell, radius);
  if (!rescued.has_value()) {
    return fail(
      PlanFailure::StartNotRescuable,
      diagnostic::rejected(
        describe_cell("start", *start_cell),
        "no Free cell within start_search_radius_cells " + std::to_string(radius)));
  }

  std::optional<eltanin::map::Costmap> corridor;
  if (parameters.planner_type == PlannerType::HybridAStar) {
    // Hybrid A* sizes its arrays from the whole map, so it searches a corridor around a raw A*
    // guide instead (eltanin docs/planner-design.md 13.7).
    eltanin::planner::AStarParams guide_params = parameters.astar;
    guide_params.smoother.reset();
    // The guide only bounds the corridor; the footprint check inside Hybrid A* decides what is
    // safe.
    guide_params.common.traversability_fallback.enabled = true;
    const eltanin::planner::PlanResult guide =
      eltanin::planner::plan_astar(costmap, model, start, goal, guide_params);
    if (!guide) {
      return fail(
        failure_of(guide.error()),
        diagnostic::line(
          "no hybrid_astar corridor from " + describe_cell("start", *start_cell) + " to " +
          describe_cell("goal", *goal_cell) + " on the " + describe_map(geometry) +
          ": the A* guide failed with " + eltanin::planner::to_string(guide.error())));
    }
    std::vector<Eigen::Vector2d> positions;
    positions.reserve(guide->size());
    for (const eltanin::Pose2D & pose : *guide) {
      positions.push_back(pose.position);
    }
    corridor =
      eltanin::map::crop_around(costmap, positions, parameters.hybrid_corridor_margin_cells);
    if (!corridor.has_value()) {
      return fail(
        PlanFailure::SearchFailed,
        diagnostic::line(
          "the A* guide left no cell of the " + describe_map(geometry) + " to crop"));
    }

    const std::size_t states =
      corridor->cell_count() * static_cast<std::size_t>(parameters.hybrid.heading_bins);
    if (states > parameters.hybrid_max_states) {
      return fail(
        PlanFailure::StateSpaceTooLarge,
        diagnostic::rejected(
          "the hybrid_astar state space",
          std::to_string(corridor->cell_count()) + " corridor cells x " +
            std::to_string(parameters.hybrid.heading_bins) + " heading bins is " +
            std::to_string(states) + " states, above hybrid.max_states " +
            std::to_string(parameters.hybrid_max_states) +
            "; raise it or lower hybrid.corridor_margin_cells " +
            std::to_string(parameters.hybrid_corridor_margin_cells)));
    }
  }

  const eltanin::planner::PlanResult result =
    parameters.planner_type == PlannerType::HybridAStar
      ? eltanin::planner::plan_hybrid_astar(*corridor, model, start, goal, parameters.hybrid)
      : eltanin::planner::plan_astar(costmap, model, start, goal, parameters.astar);
  if (!result) {
    return fail(
      failure_of(result.error()),
      diagnostic::line(
        "no path from " + describe_cell("start", *start_cell) + " to " +
        describe_cell("goal", *goal_cell) + " with " + name_of(parameters.planner_type) +
        " on the " + describe_map(geometry) + ": " + eltanin::planner::to_string(result.error())));
  }
  const eltanin::Path & path = *result;
  if (path.empty()) {
    return fail(
      PlanFailure::EmptyPath,
      diagnostic::line(
        "the search returned no poses from " + describe_cell("start", *start_cell) + " to " +
        describe_cell("goal", *goal_cell)));
  }

  const std::string message = diagnostic::line(
    std::string(name_of(parameters.planner_type)) + " planned " + std::to_string(path.size()) +
    " poses, " + std::to_string(eltanin::path_length(path)) + " m");
  return PlanAttempt{path, PlanFailure::None, message};
}

}  // namespace eltanin_planner
