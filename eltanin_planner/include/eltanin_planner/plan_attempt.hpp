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

#ifndef ELTANIN_PLANNER__PLAN_ATTEMPT_HPP_
#define ELTANIN_PLANNER__PLAN_ATTEMPT_HPP_

#include "eltanin_planner/planner_parameters.hpp"

#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>

#include <cstdint>
#include <string>

namespace eltanin_planner
{

/// The causes eltanin's plan() collapses into one nullopt; this enum is what tells them apart.
enum class PlanFailure : std::uint8_t {
  None,
  /// A resolution or cell count eltanin states with assert alone, which RelWithDebInfo drops.
  MapUnusable,
  StartOutsideMap,
  GoalOutsideMap,
  /// Reported, never moved: the goal is what the caller asked for (astar_planner's own contract).
  GoalNotFree,
  StartNotRescuable,
  /// eltanin sizes its arrays from cells * heading_bins before it expands anything.
  StateSpaceTooLarge,
  SearchFailed,
  EmptyPath
};

/// The eltanin_msgs/NavigationState OUTCOME_* each cause is reported as.
std::uint8_t to_outcome(PlanFailure failure) noexcept;

struct PlanAttempt
{
  /// Before smoothing, and empty unless failure is None.
  eltanin::Path path;
  PlanFailure failure{PlanFailure::None};
  /// Always set, always one line, and always what the action reports back as message.
  std::string message;

  bool ok() const noexcept { return failure == PlanFailure::None; }
};

/// Runs eltanin's own checks in eltanin's own order and then the search; smoothing is the caller's.
PlanAttempt attempt_plan(
  const eltanin::map::Costmap & costmap, const eltanin::map::CostTraversabilityModel & model,
  const eltanin::Pose2D & start, const eltanin::Pose2D & goal,
  const PlannerParameters & parameters);

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__PLAN_ATTEMPT_HPP_
