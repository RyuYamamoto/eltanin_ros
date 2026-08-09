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
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/types.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace eltanin_controller::composition
{

/// False while Aligning and in the two latched states: the cycle has a command without PurePursuit.
bool tracking_required(eltanin::control::GoalApproach::State state) noexcept;

/// The FollowerDiagnostic value of each enumerator; changing either enum means changing this table.
std::uint8_t to_wire(eltanin::control::FollowStatus status) noexcept;

/// Same table for the approach state; the declaration order is the wire format.
std::uint8_t to_wire(eltanin::control::GoalApproach::State state) noexcept;

/// Everything one cycle publishes except the header and the strings the node builds itself.
struct Outcome
{
  eltanin::Twist2D command{};
  std::uint8_t status{};
  std::uint8_t approach_state{};
  std::uint8_t reason{};
  /// False only for the three failures the follower cannot recover from by itself.
  bool ok{true};
  /// True only while tracking with a follower that has one; a point drawn otherwise would be zero.
  bool has_lookahead{false};
  std::size_t lookahead_index{0};
  Eigen::Vector2d lookahead_point{Eigen::Vector2d::Zero()};
};

/// The zero command of a cycle that never reached eltanin; ok stays true, the upstream can return.
Outcome input_failure(std::uint8_t reason) noexcept;

/// The whole composition, and the only caller of apply_linear_limit(); never throws, never logs.
Outcome compose(
  const eltanin::control::GoalApproach::Result & approach,
  const std::optional<eltanin::control::FollowResult> & tracking,
  const std::optional<eltanin::control::PurePursuit::Lookahead> & lookahead) noexcept;

}  // namespace eltanin_controller::composition

#endif  // ELTANIN_CONTROLLER__COMMAND_COMPOSITION_HPP_
