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

#include <eltanin_msgs/msg/navigation_state.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <set>
#include <vector>

namespace
{

using State = eltanin_msgs::msg::NavigationState;

TEST(OutcomeConstantsTest, ZeroToTenFollowTheNavigateOutcomeDeclarationOrder)
{
  EXPECT_EQ(State::OUTCOME_REACHED, 0);
  EXPECT_EQ(State::OUTCOME_MODEL_FAILED, 1);
  EXPECT_EQ(State::OUTCOME_START_GOAL_FAILED, 2);
  EXPECT_EQ(State::OUTCOME_PLAN_FAILED, 3);
  EXPECT_EQ(State::OUTCOME_PATH_TOO_SHORT, 4);
  EXPECT_EQ(State::OUTCOME_NO_PATH, 5);
  EXPECT_EQ(State::OUTCOME_GOAL_TOLERANCE_FAILED, 6);
  EXPECT_EQ(State::OUTCOME_REPLAN_FAILED, 7);
  EXPECT_EQ(State::OUTCOME_REPLAN_LIMIT, 8);
  EXPECT_EQ(State::OUTCOME_STALLED, 9);
  EXPECT_EQ(State::OUTCOME_STEP_LIMIT, 10);
}

TEST(OutcomeConstantsTest, ElevenToThirteenAreTheOnesOnlyTheRosBoundaryProduces)
{
  EXPECT_EQ(State::OUTCOME_CANCELED, 11);
  EXPECT_EQ(State::OUTCOME_TIMEOUT, 12);
  EXPECT_EQ(State::OUTCOME_INPUT_STALE, 13);
}

TEST(OutcomeConstantsTest, UnknownSitsOutsideTheEnumRange)
{
  EXPECT_EQ(State::OUTCOME_UNKNOWN, 255);
  EXPECT_GT(State::OUTCOME_UNKNOWN, State::OUTCOME_INPUT_STALE);
}

TEST(OutcomeConstantsTest, ADefaultConstructedMessageDoesNotReadAsSuccess)
{
  const State message;
  EXPECT_EQ(message.outcome, State::OUTCOME_UNKNOWN);
  EXPECT_NE(message.outcome, State::OUTCOME_REACHED);
}

TEST(OutcomeConstantsTest, AllFifteenValuesAreDistinct)
{
  const std::vector<std::uint8_t> outcomes{
    State::OUTCOME_REACHED,
    State::OUTCOME_MODEL_FAILED,
    State::OUTCOME_START_GOAL_FAILED,
    State::OUTCOME_PLAN_FAILED,
    State::OUTCOME_PATH_TOO_SHORT,
    State::OUTCOME_NO_PATH,
    State::OUTCOME_GOAL_TOLERANCE_FAILED,
    State::OUTCOME_REPLAN_FAILED,
    State::OUTCOME_REPLAN_LIMIT,
    State::OUTCOME_STALLED,
    State::OUTCOME_STEP_LIMIT,
    State::OUTCOME_CANCELED,
    State::OUTCOME_TIMEOUT,
    State::OUTCOME_INPUT_STALE,
    State::OUTCOME_UNKNOWN};
  const std::set<std::uint8_t> distinct(outcomes.begin(), outcomes.end());
  EXPECT_EQ(distinct.size(), outcomes.size());
}

TEST(StateConstantsTest, TheStateMachineHasSevenStatesNumberedZeroToSix)
{
  EXPECT_EQ(State::STATE_IDLE, 0);
  EXPECT_EQ(State::STATE_PLANNING, 1);
  EXPECT_EQ(State::STATE_FOLLOWING, 2);
  EXPECT_EQ(State::STATE_REPLANNING, 3);
  EXPECT_EQ(State::STATE_CANCELING, 4);
  EXPECT_EQ(State::STATE_SUCCEEDED, 5);
  EXPECT_EQ(State::STATE_FAILED, 6);
}

TEST(StateConstantsTest, ADefaultConstructedMessageIsIdle)
{
  const State message;
  EXPECT_EQ(message.state, State::STATE_IDLE);
}

}  // namespace
