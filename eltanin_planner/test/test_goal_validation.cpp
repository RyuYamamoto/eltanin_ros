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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/planner/path_smoother.hpp>

#include <eltanin_msgs/msg/navigation_state.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>

namespace
{

using eltanin::map::CostTraversabilityModel;
using eltanin::map::FREE_SPACE;
using eltanin::map::INSCRIBED_INFLATED_OBSTACLE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::NO_INFORMATION;
using eltanin_msgs::msg::NavigationState;
using eltanin_planner::attempt_plan;
using eltanin_planner::PlanAttempt;
using eltanin_planner::PlanFailure;
using eltanin_planner::PlannerParameters;
using eltanin_planner::to_outcome;

constexpr double RESOLUTION = 0.1;

/// Below INSCRIBED_INFLATED_OBSTACLE, so the band between it and inscribed is reachable.
constexpr std::uint8_t CIRCUMSCRIBED_COST = 100;
constexpr std::uint8_t IN_THE_CIRCUMSCRIBED_BAND = 200;

eltanin::map::Costmap make_costmap(int width = 10, int height = 10)
{
  return eltanin::map::Costmap(
    eltanin::map::MapGeometry(width, height, RESOLUTION, Eigen::Vector2d{0.0, 0.0}), FREE_SPACE);
}

CostTraversabilityModel make_model(bool unknown_is_free = false)
{
  return CostTraversabilityModel(CIRCUMSCRIBED_COST, unknown_is_free);
}

eltanin::Pose2D at_cell(const eltanin::map::Costmap & costmap, int mx, int my, double yaw = 0.0)
{
  return eltanin::Pose2D{costmap.geometry().map_to_world(mx, my), yaw};
}

::testing::AssertionResult reports_one_line(const PlanAttempt & attempt)
{
  if (attempt.message.empty()) {
    return ::testing::AssertionFailure() << "the attempt carries no message";
  }
  if (attempt.message.find('\n') != std::string::npos) {
    return ::testing::AssertionFailure() << "'" << attempt.message << "' contains a newline";
  }
  if (attempt.message.find("eltanin_planner: ") != 0) {
    return ::testing::AssertionFailure() << "'" << attempt.message << "' does not name the package";
  }
  return ::testing::AssertionSuccess();
}

TEST(ToOutcomeTest, TellsTheFourCausesEltaninCollapsesIntoOneNulloptApart)
{
  EXPECT_EQ(to_outcome(PlanFailure::None), NavigationState::OUTCOME_REACHED);
  EXPECT_EQ(to_outcome(PlanFailure::MapUnusable), NavigationState::OUTCOME_INPUT_STALE);
  EXPECT_EQ(to_outcome(PlanFailure::StartOutsideMap), NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_EQ(to_outcome(PlanFailure::GoalOutsideMap), NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_EQ(to_outcome(PlanFailure::GoalNotFree), NavigationState::OUTCOME_NO_PATH);
  EXPECT_EQ(to_outcome(PlanFailure::StartNotRescuable), NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_EQ(to_outcome(PlanFailure::SearchFailed), NavigationState::OUTCOME_PLAN_FAILED);
  EXPECT_EQ(to_outcome(PlanFailure::EmptyPath), NavigationState::OUTCOME_PLAN_FAILED);
}

TEST(AttemptPlanTest, PlansAcrossAnEmptyMap)
{
  const eltanin::map::Costmap costmap = make_costmap();
  const auto attempt = attempt_plan(
    costmap, make_model(), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8), PlannerParameters{});
  ASSERT_TRUE(attempt.ok()) << attempt.message;
  EXPECT_FALSE(attempt.path.empty());
  EXPECT_TRUE(reports_one_line(attempt));
  EXPECT_NE(attempt.message.find("poses"), std::string::npos) << attempt.message;
}

TEST(AttemptPlanTest, RejectsAGoalOutsideTheMap)
{
  const eltanin::map::Costmap costmap = make_costmap();
  const eltanin::Pose2D goal{Eigen::Vector2d{12.3, 4.5}, 0.0};
  const auto attempt =
    attempt_plan(costmap, make_model(), at_cell(costmap, 1, 1), goal, PlannerParameters{});
  EXPECT_EQ(attempt.failure, PlanFailure::GoalOutsideMap);
  EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_TRUE(reports_one_line(attempt));
  EXPECT_NE(attempt.message.find("outside the 10x10 map"), std::string::npos) << attempt.message;
  EXPECT_TRUE(attempt.path.empty());
}

TEST(AttemptPlanTest, RejectsAStartOutsideTheMap)
{
  const eltanin::map::Costmap costmap = make_costmap();
  const eltanin::Pose2D start{Eigen::Vector2d{-4.0, 0.5}, 0.0};
  const auto attempt =
    attempt_plan(costmap, make_model(), start, at_cell(costmap, 8, 8), PlannerParameters{});
  EXPECT_EQ(attempt.failure, PlanFailure::StartOutsideMap);
  EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_TRUE(reports_one_line(attempt));
}

TEST(AttemptPlanTest, ALethalGoalIsNoPathAndNotAStartGoalFailure)
{
  eltanin::map::Costmap costmap = make_costmap();
  ASSERT_TRUE(costmap.set(8, 8, LETHAL_OBSTACLE));
  const auto attempt = attempt_plan(
    costmap, make_model(), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8), PlannerParameters{});
  EXPECT_EQ(attempt.failure, PlanFailure::GoalNotFree);
  EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_NO_PATH);
  EXPECT_TRUE(reports_one_line(attempt));
  EXPECT_NE(attempt.message.find("not moved"), std::string::npos) << attempt.message;
}

TEST(AttemptPlanTest, EveryClassOtherThanFreeIsNoPath)
{
  for (const std::uint8_t cost : {INSCRIBED_INFLATED_OBSTACLE, IN_THE_CIRCUMSCRIBED_BAND}) {
    eltanin::map::Costmap costmap = make_costmap();
    ASSERT_TRUE(costmap.set(8, 8, cost));
    const auto attempt = attempt_plan(
      costmap, make_model(), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8), PlannerParameters{});
    EXPECT_EQ(attempt.failure, PlanFailure::GoalNotFree) << "cost " << static_cast<int>(cost);
    EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_NO_PATH);
  }
}

TEST(AttemptPlanTest, AnUnknownGoalFollowsUnknownIsFree)
{
  eltanin::map::Costmap costmap = make_costmap();
  ASSERT_TRUE(costmap.set(8, 8, NO_INFORMATION));
  const auto refused = attempt_plan(
    costmap, make_model(false), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8),
    PlannerParameters{});
  EXPECT_EQ(refused.failure, PlanFailure::GoalNotFree);
  EXPECT_EQ(to_outcome(refused.failure), NavigationState::OUTCOME_NO_PATH);

  const auto accepted = attempt_plan(
    costmap, make_model(true), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8), PlannerParameters{});
  EXPECT_TRUE(accepted.ok()) << accepted.message;
}

TEST(AttemptPlanTest, RejectsAStartWithNoFreeCellWithinTheSearchRadius)
{
  eltanin::map::Costmap costmap = make_costmap();
  for (int my = 0; my < 6; ++my) {
    for (int mx = 0; mx < 6; ++mx) {
      ASSERT_TRUE(costmap.set(mx, my, LETHAL_OBSTACLE));
    }
  }
  PlannerParameters parameters;
  parameters.astar.start_search_radius_cells = 2;
  const auto attempt =
    attempt_plan(costmap, make_model(), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8), parameters);
  EXPECT_EQ(attempt.failure, PlanFailure::StartNotRescuable);
  EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_TRUE(reports_one_line(attempt));
  EXPECT_NE(attempt.message.find("start_search_radius_cells 2"), std::string::npos)
    << attempt.message;
}

TEST(AttemptPlanTest, ARescuedStartBeginsThePathAtTheRescuedCell)
{
  eltanin::map::Costmap costmap = make_costmap();
  ASSERT_TRUE(costmap.set(2, 2, LETHAL_OBSTACLE));
  const auto attempt = attempt_plan(
    costmap, make_model(), at_cell(costmap, 2, 2), at_cell(costmap, 8, 8), PlannerParameters{});
  ASSERT_TRUE(attempt.ok()) << attempt.message;
  ASSERT_FALSE(attempt.path.empty());

  const auto cell = costmap.geometry().world_to_map(attempt.path[0].position);
  ASSERT_TRUE(cell.has_value());
  EXPECT_EQ(costmap.get(cell->x, cell->y), FREE_SPACE);
  EXPECT_FALSE(cell->x == 2 && cell->y == 2);
}

TEST(AttemptPlanTest, AStartSearchRadiusOfZeroDisablesTheRescue)
{
  eltanin::map::Costmap costmap = make_costmap();
  ASSERT_TRUE(costmap.set(2, 2, LETHAL_OBSTACLE));
  PlannerParameters parameters;
  parameters.astar.start_search_radius_cells = 0;
  const auto attempt =
    attempt_plan(costmap, make_model(), at_cell(costmap, 2, 2), at_cell(costmap, 8, 8), parameters);
  EXPECT_EQ(attempt.failure, PlanFailure::StartNotRescuable);
}

TEST(AttemptPlanTest, AMapCutInTwoIsAPlanFailureAndNotSomethingElse)
{
  eltanin::map::Costmap costmap = make_costmap();
  for (int my = 0; my < costmap.size_y(); ++my) {
    ASSERT_TRUE(costmap.set(5, my, LETHAL_OBSTACLE));
  }
  const auto attempt = attempt_plan(
    costmap, make_model(), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8), PlannerParameters{});
  EXPECT_EQ(attempt.failure, PlanFailure::SearchFailed);
  EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_PLAN_FAILED);
  EXPECT_TRUE(reports_one_line(attempt));
  EXPECT_NE(attempt.message.find("no path from"), std::string::npos) << attempt.message;
}

TEST(AttemptPlanTest, RejectsAMapEltaninWouldOnlyAssertOn)
{
  const eltanin::map::Costmap no_geometry;
  const auto attempt = attempt_plan(
    no_geometry, make_model(), eltanin::Pose2D{}, eltanin::Pose2D{}, PlannerParameters{});
  EXPECT_EQ(attempt.failure, PlanFailure::MapUnusable);
  EXPECT_EQ(to_outcome(attempt.failure), NavigationState::OUTCOME_INPUT_STALE);
  EXPECT_TRUE(reports_one_line(attempt));
}

TEST(AttemptPlanTest, TheRequestedGoalYawSurvivesTheSearchAndTheSmoother)
{
  const eltanin::map::Costmap costmap = make_costmap();
  const PlannerParameters parameters;
  const double goal_yaw = 1.2345;
  const auto attempt = attempt_plan(
    costmap, make_model(), at_cell(costmap, 1, 1), at_cell(costmap, 8, 8, goal_yaw), parameters);
  ASSERT_TRUE(attempt.ok()) << attempt.message;
  ASSERT_FALSE(attempt.path.empty());
  EXPECT_DOUBLE_EQ(attempt.path[attempt.path.size() - 1].yaw, goal_yaw);

  const eltanin::Path smoothed =
    eltanin::planner::smooth(attempt.path, costmap, make_model(), parameters.smoother);
  ASSERT_FALSE(smoothed.empty());
  EXPECT_DOUBLE_EQ(smoothed[smoothed.size() - 1].yaw, goal_yaw);
}

/// AC-26: run by hand with --gtest_also_run_disabled_tests, never in CI where -O0 costs 1.48 s.
TEST(AttemptPlanTest, DISABLED_PlanScale)
{
  const eltanin::map::Costmap costmap = make_costmap(4000, 4000);
  const auto started = std::chrono::steady_clock::now();
  const auto attempt = attempt_plan(
    costmap, make_model(), at_cell(costmap, 10, 10), at_cell(costmap, 3990, 3990),
    PlannerParameters{});
  const std::chrono::duration<double, std::milli> elapsed =
    std::chrono::steady_clock::now() - started;
  ASSERT_TRUE(attempt.ok()) << attempt.message;
  GTEST_LOG_(INFO) << "4000x4000: " << elapsed.count() << " ms, " << attempt.path.size()
                   << " poses";
}

}  // namespace
