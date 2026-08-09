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

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <vector>

namespace
{

using Approach = eltanin::control::GoalApproach;
using Diagnostic = eltanin_msgs::msg::FollowerDiagnostic;
using Pursuit = eltanin::control::PurePursuit;
using eltanin_controller::composition::compose;
using eltanin_controller::composition::input_failure;
using eltanin_controller::composition::Outcome;
using eltanin_controller::composition::to_wire;
using eltanin_controller::composition::tracking_required;

constexpr double INFINITE = std::numeric_limits<double>::infinity();

Approach::Result make_approach(Approach::State state, double linear_vel_limit = INFINITE)
{
  Approach::Result result;
  result.state = state;
  result.linear_vel_limit = linear_vel_limit;
  result.command.angular = 0.6;
  return result;
}

Pursuit::Result make_tracking(Pursuit::Status status, double linear = 0.5, double angular = 0.4)
{
  Pursuit::Result result;
  result.command.linear = Eigen::Vector2d{linear, 0.0};
  result.command.angular = angular;
  result.status = status;
  result.target_index = 7;
  result.lookahead_point = Eigen::Vector2d{1.25, -0.5};
  return result;
}

void expect_zero_command(const Outcome & outcome)
{
  EXPECT_EQ(outcome.command.linear.x(), 0.0);
  EXPECT_EQ(outcome.command.linear.y(), 0.0);
  EXPECT_EQ(outcome.command.angular, 0.0);
  EXPECT_FALSE(outcome.has_lookahead);
  EXPECT_EQ(outcome.lookahead_index, 0u);
}

TEST(TrackingRequiredTest, OnlyTheTwoStatesThatStillDriveNeedPurePursuit)
{
  EXPECT_TRUE(tracking_required(Approach::State::Inactive));
  EXPECT_TRUE(tracking_required(Approach::State::Approaching));
  EXPECT_FALSE(tracking_required(Approach::State::Aligning));
  EXPECT_FALSE(tracking_required(Approach::State::Reached));
  EXPECT_FALSE(tracking_required(Approach::State::AlignmentTimeout));
}

TEST(ComposeTest, AligningIgnoresWhateverPurePursuitWouldHaveSaid)
{
  const Approach::Result approach = make_approach(Approach::State::Aligning);
  const Outcome without = compose(approach, std::nullopt);
  const Outcome with = compose(approach, make_tracking(Pursuit::Status::Tracking, 9.0, -9.0));

  EXPECT_EQ(without.command.angular, approach.command.angular);
  EXPECT_EQ(with.command.angular, approach.command.angular);
  EXPECT_EQ(with.command.linear.x(), 0.0);
  EXPECT_FALSE(with.has_lookahead);
  EXPECT_EQ(with.status, Diagnostic::STATUS_NO_PATH);
  EXPECT_EQ(with.approach_state, Diagnostic::APPROACH_ALIGNING);
  EXPECT_EQ(with.reason, Diagnostic::REASON_NONE);
  EXPECT_TRUE(with.ok);
}

TEST(ComposeTest, ReachedIsAZeroCommandAndNotAFailure)
{
  const Outcome outcome = compose(make_approach(Approach::State::Reached, 0.0), std::nullopt);
  expect_zero_command(outcome);
  EXPECT_EQ(outcome.approach_state, Diagnostic::APPROACH_REACHED);
  EXPECT_EQ(outcome.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_TRUE(outcome.ok);
}

TEST(ComposeTest, AlignmentTimeoutIsAZeroCommandAndAFailure)
{
  const Outcome outcome =
    compose(make_approach(Approach::State::AlignmentTimeout, 0.0), std::nullopt);
  expect_zero_command(outcome);
  EXPECT_EQ(outcome.approach_state, Diagnostic::APPROACH_ALIGNMENT_TIMEOUT);
  EXPECT_EQ(outcome.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_FALSE(outcome.ok);
}

TEST(ComposeTest, AnUnlimitedApproachLeavesTheTrackingCommandUntouched)
{
  const Pursuit::Result tracking = make_tracking(Pursuit::Status::Tracking);
  const Outcome outcome = compose(make_approach(Approach::State::Inactive, INFINITE), tracking);

  EXPECT_EQ(outcome.command.linear.x(), tracking.command.linear.x());
  EXPECT_EQ(outcome.command.angular, tracking.command.angular);
  EXPECT_EQ(outcome.status, Diagnostic::STATUS_TRACKING);
  EXPECT_EQ(outcome.approach_state, Diagnostic::APPROACH_INACTIVE);
  EXPECT_EQ(outcome.reason, Diagnostic::REASON_NONE);
  EXPECT_TRUE(outcome.ok);
  EXPECT_TRUE(outcome.has_lookahead);
  EXPECT_EQ(outcome.lookahead_index, tracking.target_index);
  EXPECT_EQ(outcome.lookahead_point, tracking.lookahead_point);
}

TEST(ComposeTest, TheApproachLimitScalesTheWholeCommandAndKeepsTheCurvature)
{
  const Pursuit::Result tracking = make_tracking(Pursuit::Status::Tracking, 0.5, 0.4);
  const Outcome outcome = compose(make_approach(Approach::State::Approaching, 0.25), tracking);

  EXPECT_DOUBLE_EQ(outcome.command.linear.x(), 0.25);
  // std::min on linear.x alone would leave 0.4 here, and the curvature would double.
  EXPECT_DOUBLE_EQ(outcome.command.angular, 0.2);
  EXPECT_DOUBLE_EQ(
    outcome.command.angular / outcome.command.linear.x(),
    tracking.command.angular / tracking.command.linear.x());
  EXPECT_EQ(outcome.approach_state, Diagnostic::APPROACH_APPROACHING);
}

TEST(ComposeTest, GoalReachedBeforeTheApproachAcceptedItIsAFailure)
{
  const Outcome outcome = compose(
    make_approach(Approach::State::Approaching, 0.4), make_tracking(Pursuit::Status::GoalReached));
  expect_zero_command(outcome);
  EXPECT_EQ(outcome.status, Diagnostic::STATUS_GOAL_REACHED);
  EXPECT_EQ(outcome.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_FALSE(outcome.ok);
}

TEST(ComposeTest, NoPathIsAFailureEvenThoughTheNodeFiltersEmptyPathsFirst)
{
  const Outcome outcome =
    compose(make_approach(Approach::State::Inactive), make_tracking(Pursuit::Status::NoPath));
  expect_zero_command(outcome);
  EXPECT_EQ(outcome.status, Diagnostic::STATUS_NO_PATH);
  EXPECT_EQ(outcome.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_FALSE(outcome.ok);
}

#ifdef NDEBUG
TEST(ComposeTest, AMissingTrackingResultDegradesToTheSafeSide)
{
  const Outcome outcome = compose(make_approach(Approach::State::Inactive), std::nullopt);
  expect_zero_command(outcome);
  EXPECT_EQ(outcome.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_FALSE(outcome.ok);
}
#endif

TEST(InputFailureTest, EveryInputSideReasonIsAZeroCommandThatIsStillOk)
{
  const std::vector<std::uint8_t> reasons{
    Diagnostic::REASON_NO_INPUT,     Diagnostic::REASON_INPUT_STALE,
    Diagnostic::REASON_INPUT_EMPTY,  Diagnostic::REASON_INPUT_REJECTED,
    Diagnostic::REASON_NO_TRANSFORM, Diagnostic::REASON_NO_DT};
  for (const std::uint8_t reason : reasons) {
    const Outcome outcome = input_failure(reason);
    expect_zero_command(outcome);
    EXPECT_EQ(outcome.reason, reason);
    EXPECT_EQ(outcome.status, Diagnostic::STATUS_NO_PATH);
    EXPECT_EQ(outcome.approach_state, Diagnostic::APPROACH_INACTIVE);
    EXPECT_TRUE(outcome.ok) << "reason " << static_cast<int>(reason);
  }
}

TEST(WireConstantsTest, ThePursuitStatusFollowsItsDeclarationOrder)
{
  EXPECT_EQ(to_wire(Pursuit::Status::NoPath), Diagnostic::STATUS_NO_PATH);
  EXPECT_EQ(to_wire(Pursuit::Status::Tracking), Diagnostic::STATUS_TRACKING);
  EXPECT_EQ(to_wire(Pursuit::Status::GoalReached), Diagnostic::STATUS_GOAL_REACHED);
  EXPECT_EQ(Diagnostic::STATUS_NO_PATH, 0);
  EXPECT_EQ(Diagnostic::STATUS_TRACKING, 1);
  EXPECT_EQ(Diagnostic::STATUS_GOAL_REACHED, 2);
}

TEST(WireConstantsTest, TheApproachStateFollowsItsDeclarationOrder)
{
  EXPECT_EQ(to_wire(Approach::State::Inactive), Diagnostic::APPROACH_INACTIVE);
  EXPECT_EQ(to_wire(Approach::State::Approaching), Diagnostic::APPROACH_APPROACHING);
  EXPECT_EQ(to_wire(Approach::State::Aligning), Diagnostic::APPROACH_ALIGNING);
  EXPECT_EQ(to_wire(Approach::State::Reached), Diagnostic::APPROACH_REACHED);
  EXPECT_EQ(to_wire(Approach::State::AlignmentTimeout), Diagnostic::APPROACH_ALIGNMENT_TIMEOUT);
  EXPECT_EQ(Diagnostic::APPROACH_INACTIVE, 0);
  EXPECT_EQ(Diagnostic::APPROACH_APPROACHING, 1);
  EXPECT_EQ(Diagnostic::APPROACH_ALIGNING, 2);
  EXPECT_EQ(Diagnostic::APPROACH_REACHED, 3);
  EXPECT_EQ(Diagnostic::APPROACH_ALIGNMENT_TIMEOUT, 4);
}

TEST(WireConstantsTest, TheEightReasonsAreDistinctAndNoneIsZeroTwice)
{
  const std::vector<std::uint8_t> reasons{
    Diagnostic::REASON_NONE,           Diagnostic::REASON_NO_INPUT,
    Diagnostic::REASON_INPUT_STALE,    Diagnostic::REASON_INPUT_EMPTY,
    Diagnostic::REASON_INPUT_REJECTED, Diagnostic::REASON_NO_TRANSFORM,
    Diagnostic::REASON_NO_DT,          Diagnostic::REASON_CONTROLLER};
  const std::set<std::uint8_t> distinct(reasons.begin(), reasons.end());
  EXPECT_EQ(distinct.size(), reasons.size());
  EXPECT_EQ(Diagnostic::REASON_NONE, 0);
}

}  // namespace
