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

#include "eltanin_controller/command_composition.hpp"

#include <eltanin_msgs/msg/follower_diagnostic.hpp>

#include <cassert>

namespace eltanin_controller::composition
{

namespace
{

using Diagnostic = eltanin_msgs::msg::FollowerDiagnostic;
using Approach = eltanin::control::GoalApproach;
using Pursuit = eltanin::control::PurePursuit;

/// The zero command the follower falls back to; eltanin returns it too, and never a stale one.
Outcome controller_failure(const Approach::Result & approach, Pursuit::Status status, bool ok)
{
  Outcome outcome;
  outcome.status = to_wire(status);
  outcome.approach_state = to_wire(approach.state);
  outcome.reason = Diagnostic::REASON_CONTROLLER;
  outcome.ok = ok;
  return outcome;
}

}  // namespace

bool tracking_required(Approach::State state) noexcept
{
  return state == Approach::State::Inactive || state == Approach::State::Approaching;
}

std::uint8_t to_wire(Pursuit::Status status) noexcept
{
  switch (status) {
    case Pursuit::Status::NoPath:
      return Diagnostic::STATUS_NO_PATH;
    case Pursuit::Status::Tracking:
      return Diagnostic::STATUS_TRACKING;
    case Pursuit::Status::GoalReached:
      return Diagnostic::STATUS_GOAL_REACHED;
  }
  return Diagnostic::STATUS_NO_PATH;
}

std::uint8_t to_wire(Approach::State state) noexcept
{
  switch (state) {
    case Approach::State::Inactive:
      return Diagnostic::APPROACH_INACTIVE;
    case Approach::State::Approaching:
      return Diagnostic::APPROACH_APPROACHING;
    case Approach::State::Aligning:
      return Diagnostic::APPROACH_ALIGNING;
    case Approach::State::Reached:
      return Diagnostic::APPROACH_REACHED;
    case Approach::State::AlignmentTimeout:
      return Diagnostic::APPROACH_ALIGNMENT_TIMEOUT;
  }
  return Diagnostic::APPROACH_INACTIVE;
}

Outcome input_failure(std::uint8_t reason) noexcept
{
  assert(reason != Diagnostic::REASON_NONE && reason != Diagnostic::REASON_CONTROLLER);
  Outcome outcome;
  outcome.status = Diagnostic::STATUS_NO_PATH;
  outcome.approach_state = Diagnostic::APPROACH_INACTIVE;
  outcome.reason = reason;
  return outcome;
}

Outcome compose(
  const Approach::Result & approach, const std::optional<Pursuit::Result> & tracking) noexcept
{
  if (approach.state == Approach::State::Reached) {
    return controller_failure(approach, Pursuit::Status::NoPath, true);
  }
  if (approach.state == Approach::State::AlignmentTimeout) {
    return controller_failure(approach, Pursuit::Status::NoPath, false);
  }
  if (approach.state == Approach::State::Aligning) {
    Outcome outcome;
    outcome.command = approach.command;
    outcome.status = to_wire(Pursuit::Status::NoPath);
    outcome.approach_state = to_wire(approach.state);
    outcome.reason = Diagnostic::REASON_NONE;
    return outcome;
  }

  // Calling with nullopt here is a contract violation; the assert says so and the fallback is safe.
  assert(tracking.has_value());
  if (!tracking.has_value()) {
    return controller_failure(approach, Pursuit::Status::NoPath, false);
  }
  if (tracking->status != Pursuit::Status::Tracking) {
    return controller_failure(approach, tracking->status, false);
  }

  Outcome outcome;
  outcome.command =
    eltanin::control::detail::apply_linear_limit(tracking->command, approach.linear_vel_limit);
  outcome.status = to_wire(tracking->status);
  outcome.approach_state = to_wire(approach.state);
  outcome.reason = Diagnostic::REASON_NONE;
  outcome.has_lookahead = true;
  outcome.lookahead_index = tracking->target_index;
  outcome.lookahead_point = tracking->lookahead_point;
  return outcome;
}

}  // namespace eltanin_controller::composition
