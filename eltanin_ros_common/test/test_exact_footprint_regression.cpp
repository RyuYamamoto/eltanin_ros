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

#include <eltanin/collision/collision_checker.hpp>
#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/map_geometry.hpp>

#include <gtest/gtest.h>

namespace eltanin_ros_common::test
{
namespace
{

constexpr double RESOLUTION = 0.05;
constexpr int LETHAL_CELL_X = 13;
constexpr int LETHAL_CELL_Y = 11;

/// A 2 m square with no inflation at all: the case the cheap first stage is not allowed to answer.
eltanin::map::Costmap uninflated_map()
{
  const eltanin::map::MapGeometry geometry(40, 40, RESOLUTION, Eigen::Vector2d::Zero());
  eltanin::map::Costmap map(geometry, eltanin::map::FREE_SPACE);
  EXPECT_TRUE(map.set(LETHAL_CELL_X, LETHAL_CELL_Y, eltanin::map::LETHAL_OBSTACLE));
  return map;
}

/// The kachaka footprint of design section 7, the same shape robot_profile defaults to.
eltanin::Polygon2D kachaka_footprint()
{
  return eltanin::Polygon2D{
    Eigen::Vector2d{-0.150, -0.120}, Eigen::Vector2d{0.237, -0.120}, Eigen::Vector2d{0.237, 0.120},
    Eigen::Vector2d{-0.150, 0.120}};
}

/// Everything below LETHAL is free here, which is what "not inflated" means for the cost model.
eltanin::map::CostTraversabilityModel uninflated_model()
{
  return eltanin::map::CostTraversabilityModel(eltanin::map::INSCRIBED_INFLATED_OBSTACLE);
}

TEST(ExactFootprintRegressionTest, ExactCheckIsOnByDefault)
{
  EXPECT_TRUE(eltanin::collision::VelocityLimiterParams{}.exact_footprint_check);
}

TEST(ExactFootprintRegressionTest, TheOverlapIsRealAndTheCentreCellIsFree)
{
  const eltanin::map::Costmap map = uninflated_map();
  const eltanin::Pose2D pose{Eigen::Vector2d{0.500, 0.500}, 0.0};

  const Eigen::Vector2d obstacle = map.geometry().map_to_world(LETHAL_CELL_X, LETHAL_CELL_Y);

  EXPECT_TRUE(eltanin::contains(eltanin::transform(kachaka_footprint(), pose), obstacle));
  EXPECT_EQ(map.get(10, 10), eltanin::map::FREE_SPACE);
}

TEST(ExactFootprintRegressionTest, ExactCheckSeesTheOverlapTheCentreCellMisses)
{
  const eltanin::map::Costmap map = uninflated_map();
  const eltanin::map::CostTraversabilityModel model = uninflated_model();
  const eltanin::Pose2D pose{Eigen::Vector2d{0.500, 0.500}, 0.0};

  const eltanin::collision::CollisionCheck exact =
    eltanin::collision::check_footprint_exact(map, model, kachaka_footprint(), pose);
  const eltanin::collision::CollisionCheck inexact =
    eltanin::collision::check_footprint(map, model, kachaka_footprint(), pose);

  EXPECT_EQ(exact, eltanin::collision::CollisionCheck::Collision);
  EXPECT_EQ(inexact, eltanin::collision::CollisionCheck::Free);
}

TEST(ExactFootprintRegressionTest, LimiterLimitsTheCommandTowardsIt)
{
  const eltanin::map::Costmap map = uninflated_map();
  const eltanin::map::CostTraversabilityModel model = uninflated_model();
  eltanin::collision::VelocityLimiterParams params;
  params.footprint = kachaka_footprint();
  const auto limiter = eltanin::collision::VelocityLimiter::create(params);
  ASSERT_TRUE(limiter.has_value());

  const eltanin::Pose2D start{Eigen::Vector2d{0.300, 0.500}, 0.0};
  const eltanin::Twist2D command{Eigen::Vector2d{0.2, 0.0}, 0.0};
  ASSERT_EQ(
    eltanin::collision::check_footprint_exact(map, model, params.footprint, start),
    eltanin::collision::CollisionCheck::Free);

  const eltanin::collision::VelocityLimiter::Result result =
    limiter->limit(map, model, start, command);

  EXPECT_TRUE(result.has_collision);
  EXPECT_LT(result.command.linear.x(), 0.2);
}

}  // namespace
}  // namespace eltanin_ros_common::test
