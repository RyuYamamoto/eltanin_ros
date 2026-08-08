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

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace
{

using eltanin_planner::PlannerParameters;
using eltanin_planner::validate;

constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

::testing::AssertionResult rejects(const std::string & message, const std::string & fragment)
{
  if (message.empty()) {
    return ::testing::AssertionFailure() << "the parameters were accepted";
  }
  if (message.find('\n') != std::string::npos) {
    return ::testing::AssertionFailure() << "'" << message << "' contains a newline";
  }
  if (message.find("eltanin_planner: rejected ") != 0) {
    return ::testing::AssertionFailure() << "'" << message << "' is not shaped as a rejection";
  }
  if (message.find(fragment) == std::string::npos) {
    return ::testing::AssertionFailure()
           << "'" << message << "' does not contain '" << fragment << "'";
  }
  return ::testing::AssertionSuccess();
}

TEST(PlannerParametersTest, TheDefaultsAreEltaninsOwnDefaults)
{
  const PlannerParameters parameters;
  EXPECT_EQ(parameters.astar.common.start_search_radius_cells, 8);
  EXPECT_DOUBLE_EQ(parameters.smoother.weight_data, 0.1);
  EXPECT_DOUBLE_EQ(parameters.smoother.weight_smooth, 0.4);
  EXPECT_DOUBLE_EQ(parameters.smoother.tolerance, 1e-4);
  EXPECT_EQ(parameters.smoother.max_iterations, 100);
  EXPECT_FALSE(parameters.publish_raw_path);
  EXPECT_FALSE(parameters.unknown_is_free);
  EXPECT_DOUBLE_EQ(parameters.tf_lookup_timeout, 0.1);
}

TEST(PlannerParametersTest, TheDefaultsAreAccepted)
{
  const auto status = validate(PlannerParameters{});
  EXPECT_TRUE(status.ok()) << status.message();
}

TEST(PlannerParametersTest, RejectsANegativeStartSearchRadius)
{
  PlannerParameters parameters;
  parameters.astar.common.start_search_radius_cells = -1;
  EXPECT_TRUE(rejects(validate(parameters).message(), "start_search_radius_cells"));
}

TEST(PlannerParametersTest, AcceptsAZeroStartSearchRadiusWhichDisablesTheRescue)
{
  PlannerParameters parameters;
  parameters.astar.common.start_search_radius_cells = 0;
  EXPECT_TRUE(validate(parameters).ok());
}

TEST(PlannerParametersTest, RejectsANegativeOrNonFiniteWeightData)
{
  PlannerParameters negative;
  negative.smoother.weight_data = -0.1;
  EXPECT_TRUE(rejects(validate(negative).message(), "weight_data"));

  PlannerParameters not_a_number;
  not_a_number.smoother.weight_data = NOT_A_NUMBER;
  EXPECT_TRUE(rejects(validate(not_a_number).message(), "must be finite"));
}

TEST(PlannerParametersTest, RejectsANegativeOrNonFiniteWeightSmooth)
{
  PlannerParameters negative;
  negative.smoother.weight_smooth = -0.1;
  EXPECT_TRUE(rejects(validate(negative).message(), "weight_smooth"));

  PlannerParameters infinite;
  infinite.smoother.weight_smooth = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(rejects(validate(infinite).message(), "must be finite"));
}

TEST(PlannerParametersTest, RejectsANegativeOrNonFiniteSmootherTolerance)
{
  PlannerParameters negative;
  negative.smoother.tolerance = -1e-6;
  EXPECT_TRUE(rejects(validate(negative).message(), "smoother_tolerance"));

  PlannerParameters not_a_number;
  not_a_number.smoother.tolerance = NOT_A_NUMBER;
  EXPECT_TRUE(rejects(validate(not_a_number).message(), "smoother_tolerance"));
}

TEST(PlannerParametersTest, RejectsNegativeSmootherIterations)
{
  PlannerParameters parameters;
  parameters.smoother.max_iterations = -1;
  EXPECT_TRUE(rejects(validate(parameters).message(), "smoother_max_iterations"));
}

TEST(PlannerParametersTest, RejectsANegativeOrNonFiniteTfLookupTimeout)
{
  PlannerParameters negative;
  negative.tf_lookup_timeout = -0.1;
  EXPECT_TRUE(rejects(validate(negative).message(), "tf_lookup_timeout"));

  PlannerParameters not_a_number;
  not_a_number.tf_lookup_timeout = NOT_A_NUMBER;
  EXPECT_TRUE(rejects(validate(not_a_number).message(), "tf_lookup_timeout"));
}

TEST(PlannerParametersTest, RejectsTheWeightsThatMakeTheSmootherDiverge)
{
  PlannerParameters parameters;
  parameters.smoother.weight_data = 0.5;
  parameters.smoother.weight_smooth = 0.375;
  const auto at_the_bound = validate(parameters);
  EXPECT_TRUE(rejects(at_the_bound.message(), "the smoother diverges"));
  EXPECT_TRUE(rejects(at_the_bound.message(), "weight_smooth"));

  parameters.smoother.weight_smooth = 0.3749;
  EXPECT_TRUE(validate(parameters).ok());
}

TEST(PlannerParametersTest, TheHybridDefaultsAreEltaninsOwnDefaults)
{
  const PlannerParameters parameters;
  EXPECT_EQ(parameters.planner_type, eltanin_planner::PlannerType::AStar);
  EXPECT_EQ(parameters.hybrid.common.start_search_radius_cells, 8);
  EXPECT_EQ(parameters.hybrid.heading_bins, 72);
  EXPECT_DOUBLE_EQ(parameters.hybrid.minimum_turning_radius, 0.4);
  EXPECT_DOUBLE_EQ(parameters.hybrid.motion_step, 0.0);
  EXPECT_DOUBLE_EQ(parameters.hybrid.collision_check_step, 0.0);
  EXPECT_DOUBLE_EQ(parameters.hybrid.dubins_expansion_distance, 1.0);
  EXPECT_DOUBLE_EQ(parameters.hybrid.steering_penalty, 0.05);
  EXPECT_DOUBLE_EQ(parameters.hybrid.steering_change_penalty, 0.10);
  EXPECT_EQ(parameters.hybrid.max_expansions, 4000000u);
  EXPECT_DOUBLE_EQ(parameters.hybrid.analytic_expansion_ratio, 1.0);
  EXPECT_FALSE(parameters.hybrid.free_goal_yaw);
  EXPECT_DOUBLE_EQ(parameters.hybrid.heuristic_weight, 0.8);
  EXPECT_FALSE(parameters.publish_footprint_path);
  EXPECT_EQ(parameters.footprint_marker_stride, 10);
  EXPECT_EQ(parameters.hybrid_max_states, 20000000u);
}

TEST(PlannerParametersTest, ThePlannerTypeNamesRoundTrip)
{
  using eltanin_planner::name_of;
  using eltanin_planner::PlannerType;
  using eltanin_planner::to_planner_type;
  EXPECT_STREQ(name_of(PlannerType::AStar), "astar");
  EXPECT_STREQ(name_of(PlannerType::HybridAStar), "hybrid_astar");
  EXPECT_EQ(to_planner_type("astar"), PlannerType::AStar);
  EXPECT_EQ(to_planner_type("hybrid_astar"), PlannerType::HybridAStar);
  EXPECT_FALSE(to_planner_type("dubins").has_value());
  EXPECT_FALSE(to_planner_type("").has_value());
}

TEST(PlannerParametersTest, RejectsTheHybridValuesEltaninWouldThrowOn)
{
  PlannerParameters few_bins;
  few_bins.hybrid.heading_bins = 7;
  EXPECT_TRUE(rejects(validate(few_bins).message(), "hybrid.heading_bins"));

  PlannerParameters no_radius;
  no_radius.hybrid.minimum_turning_radius = 0.0;
  EXPECT_TRUE(rejects(validate(no_radius).message(), "hybrid.minimum_turning_radius"));

  PlannerParameters negative_step;
  negative_step.hybrid.motion_step = -0.1;
  EXPECT_TRUE(rejects(validate(negative_step).message(), "hybrid.motion_step"));

  PlannerParameters negative_check;
  negative_check.hybrid.collision_check_step = -0.1;
  EXPECT_TRUE(rejects(validate(negative_check).message(), "hybrid.collision_check_step"));

  PlannerParameters no_dubins;
  no_dubins.hybrid.dubins_expansion_distance = 0.0;
  EXPECT_TRUE(rejects(validate(no_dubins).message(), "hybrid.dubins_expansion_distance"));

  PlannerParameters negative_steering;
  negative_steering.hybrid.steering_penalty = -0.1;
  EXPECT_TRUE(rejects(validate(negative_steering).message(), "hybrid.steering_penalty"));

  PlannerParameters negative_change;
  negative_change.hybrid.steering_change_penalty = NOT_A_NUMBER;
  EXPECT_TRUE(rejects(validate(negative_change).message(), "hybrid.steering_change_penalty"));
}

TEST(PlannerParametersTest, TheHybridValuesAreCheckedEvenWhileAStarIsSelected)
{
  PlannerParameters parameters;
  ASSERT_EQ(parameters.planner_type, eltanin_planner::PlannerType::AStar);
  parameters.hybrid.minimum_turning_radius = -1.0;
  EXPECT_TRUE(rejects(validate(parameters).message(), "hybrid.minimum_turning_radius"));
}

TEST(PlannerParametersTest, RejectsAFootprintStrideBelowOne)
{
  PlannerParameters parameters;
  parameters.footprint_marker_stride = 0;
  EXPECT_TRUE(rejects(validate(parameters).message(), "footprint_marker_stride"));
}

TEST(PlannerParametersTest, RejectsAStateSpaceCeilingOfZero)
{
  PlannerParameters parameters;
  parameters.hybrid_max_states = 0;
  EXPECT_TRUE(rejects(validate(parameters).message(), "hybrid.max_states"));
}

TEST(PlannerParametersTest, ReportsOnlyTheFirstViolatedCondition)
{
  PlannerParameters parameters;
  parameters.astar.common.start_search_radius_cells = -1;
  parameters.smoother.weight_data = -1.0;
  parameters.tf_lookup_timeout = -1.0;
  const std::string message = validate(parameters).message();
  EXPECT_TRUE(rejects(message, "start_search_radius_cells"));
  EXPECT_EQ(message.find("weight_data"), std::string::npos) << message;
  EXPECT_EQ(message.find("tf_lookup_timeout"), std::string::npos) << message;
}

}  // namespace
