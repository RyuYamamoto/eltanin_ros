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

#ifndef ELTANIN_CONTROLLER__COMMAND_COMPOSITION_HPP_
#define ELTANIN_CONTROLLER__COMMAND_COMPOSITION_HPP_

#include <Eigen/Core>
#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/path_follower.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace eltanin_controller::composition
{

/// False while Aligning and in the two latched states: the cycle has a command without PurePursuit.
bool tracking_required(eltanin::control::GoalApproach::State state) noexcept;

/// Why the command is what it is; anything but None means eltanin was not asked or did not track.
enum class FollowerReason {
  None,           ///< eltanin returned a tracking command
  NoInput,        ///< no path has ever arrived
  InputStale,     ///< the deadline on the path elapsed
  InputEmpty,     ///< the path carries no poses
  InputRejected,  ///< wrong frame_id, or the conversion refused it
  NoTransform,    ///< frames.map to frames.base could not be looked up
  NoDt,           ///< no usable elapsed time this cycle, the first one included
  Controller      ///< eltanin returned something other than tracking; read status and approach
};

constexpr const char * name_of(FollowerReason reason) noexcept
{
  switch (reason) {
    case FollowerReason::None:
      return "none";
    case FollowerReason::NoInput:
      return "no_input";
    case FollowerReason::InputStale:
      return "input_stale";
    case FollowerReason::InputEmpty:
      return "input_empty";
    case FollowerReason::InputRejected:
      return "input_rejected";
    case FollowerReason::NoTransform:
      return "no_transform";
    case FollowerReason::NoDt:
      return "no_dt";
    case FollowerReason::Controller:
      return "controller";
  }
  return "unknown";
}

constexpr const char * name_of(eltanin::control::GoalApproach::State state) noexcept
{
  switch (state) {
    case eltanin::control::GoalApproach::State::Inactive:
      return "inactive";
    case eltanin::control::GoalApproach::State::Approaching:
      return "approaching";
    case eltanin::control::GoalApproach::State::Aligning:
      return "aligning";
    case eltanin::control::GoalApproach::State::Reached:
      return "reached";
    case eltanin::control::GoalApproach::State::AlignmentTimeout:
      return "alignment_timeout";
  }
  return "unknown";
}

constexpr const char * name_of(eltanin::Direction direction) noexcept
{
  switch (direction) {
    case eltanin::Direction::Forward:
      return "forward";
    case eltanin::Direction::Reverse:
      return "reverse";
    case eltanin::Direction::InPlace:
      return "in_place";
  }
  return "unknown";
}

/// Everything one cycle publishes except the header and the strings the node builds itself.
struct Outcome
{
  eltanin::Twist2D command{};
  eltanin::control::FollowStatus status{eltanin::control::FollowStatus::NoPath};
  eltanin::control::GoalApproach::State approach_state{
    eltanin::control::GoalApproach::State::Inactive};
  FollowerReason reason{FollowerReason::None};
  /// False only for the three failures the follower cannot recover from by itself.
  bool ok{true};
  /// True only while tracking with a follower that has one; a point drawn otherwise would be zero.
  bool has_lookahead{false};
  std::size_t lookahead_index{0};
  Eigen::Vector2d lookahead_point{Eigen::Vector2d::Zero()};
};

/// The zero command of a cycle that never reached eltanin; ok stays true, the upstream can return.
Outcome input_failure(FollowerReason reason) noexcept;

/// DiagnosticStatus level; ERROR exactly when ok is false, which is the navigator stopping trigger.
std::uint8_t level_of(const Outcome & outcome) noexcept;

/// The whole composition, and the only caller of apply_linear_limit(); never throws, never logs.
Outcome compose(
  const eltanin::control::GoalApproach::Result & approach,
  const std::optional<eltanin::control::FollowResult> & tracking,
  const std::optional<eltanin::control::PurePursuit::Lookahead> & lookahead) noexcept;

}  // namespace eltanin_controller::composition

#endif  // ELTANIN_CONTROLLER__COMMAND_COMPOSITION_HPP_
