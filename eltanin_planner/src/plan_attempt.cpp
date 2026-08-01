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

#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <eltanin_msgs/msg/navigation_state.hpp>

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>

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

PlanAttempt fail(PlanFailure failure, std::string message)
{
  return PlanAttempt{eltanin::Path{}, failure, std::move(message)};
}

/// Both searches rescue the start themselves, and the pre-check has to use the same radius.
int search_radius(const PlannerParameters & parameters) noexcept
{
  return parameters.planner_type == PlannerType::HybridAStar
           ? parameters.hybrid.start_search_radius_cells
           : parameters.astar.start_search_radius_cells;
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

  if (parameters.planner_type == PlannerType::HybridAStar) {
    const std::size_t states =
      costmap.cell_count() * static_cast<std::size_t>(parameters.hybrid.heading_bins);
    if (states > parameters.hybrid_max_states) {
      return fail(
        PlanFailure::StateSpaceTooLarge,
        diagnostic::rejected(
          "the hybrid_astar state space",
          std::to_string(costmap.cell_count()) + " cells x " +
            std::to_string(parameters.hybrid.heading_bins) + " heading bins is " +
            std::to_string(states) + " states, above hybrid.max_states " +
            std::to_string(parameters.hybrid_max_states) +
            "; the search allocates all of them before it expands anything"));
    }
  }

  std::optional<eltanin::Path> path =
    parameters.planner_type == PlannerType::HybridAStar
      ? eltanin::planner::plan_hybrid_astar(costmap, model, start, goal, parameters.hybrid)
      : eltanin::planner::plan(costmap, model, start, goal, parameters.astar);
  if (!path.has_value()) {
    return fail(
      PlanFailure::SearchFailed,
      diagnostic::line(
        "no path from " + describe_cell("start", *start_cell) + " to " +
        describe_cell("goal", *goal_cell) + " with " + name_of(parameters.planner_type) +
        " on the " + describe_map(geometry)));
  }
  if (path->empty()) {
    return fail(
      PlanFailure::EmptyPath,
      diagnostic::line(
        "the search returned no poses from " + describe_cell("start", *start_cell) + " to " +
        describe_cell("goal", *goal_cell)));
  }

  const std::string message = diagnostic::line(
    std::string(name_of(parameters.planner_type)) + " planned " + std::to_string(path->size()) +
    " poses, " + std::to_string(eltanin::path_length(*path)) + " m");
  return PlanAttempt{std::move(*path), PlanFailure::None, message};
}

}  // namespace eltanin_planner
