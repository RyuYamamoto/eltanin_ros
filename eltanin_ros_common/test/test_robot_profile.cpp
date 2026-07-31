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

#include "eltanin_ros_common/robot_profile.hpp"
#include "test/conversion_test_helpers.hpp"

#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/cost_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace eltanin_ros_common::test
{
namespace
{

const double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

/// The kachaka footprint of design section 7, which is also the code default (A-42).
std::vector<double> kachaka_footprint()
{
  return {-0.150, -0.120, 0.237, -0.120, 0.237, 0.120, -0.150, 0.120};
}

/// The only test file in this package that constructs a node, and so the only one calling init.
class RobotProfileTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

protected:
  /// Overrides stand in for the /** wildcard: only the key names have to match (A-26).
  std::shared_ptr<rclcpp::Node> make_node(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    ++nodes_made_;
    return std::make_shared<rclcpp::Node>(
      "test_robot_profile_" + std::to_string(nodes_made_), options);
  }

  std::shared_ptr<rclcpp::Node> make_node() { return make_node({}); }

  void expect_rejected(
    const ConversionResult<RobotProfile> & result, const std::string & key,
    const std::string & value)
  {
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(contains(result.error(), key));
    EXPECT_TRUE(contains(result.error(), value));
    EXPECT_TRUE(is_one_line(result.error()));
    EXPECT_TRUE(names_the_package_once(result.error()));
  }

private:
  int nodes_made_{0};
};

TEST_F(RobotProfileTest, DefaultsAloneGiveAProfile)
{
  const auto node = make_node();

  const ConversionResult<RobotProfile> result = declare_robot_profile(*node);

  EXPECT_TRUE(result.ok()) << result.error();
}

TEST_F(RobotProfileTest, DefaultsAreTheKachakaFootprint)
{
  const auto node = make_node();

  const ConversionResult<RobotProfile> result = declare_robot_profile(*node);

  ASSERT_TRUE(result.ok()) << result.error();
  const RobotProfile & profile = result.value();
  EXPECT_EQ(profile.footprint().size(), 4u);
  EXPECT_NEAR(profile.radii().inscribed_radius(), 0.120, 1e-9);
  EXPECT_NEAR(profile.radii().circumscribed_radius(), 0.26565, 1e-5);
  EXPECT_NEAR(profile.radii().inflation_radius(), 0.55, 1e-9);
  EXPECT_NEAR(profile.cost_scaling_factor(), 10.0, 1e-9);
  EXPECT_NEAR(profile.limits().max_linear_vel, 0.30, 1e-9);
  EXPECT_NEAR(profile.limits().max_angular_vel, 1.57, 1e-9);
  EXPECT_NEAR(profile.limits().max_accel, 0.5, 1e-9);
  EXPECT_NEAR(profile.limits().max_decel, 0.5, 1e-9);
}

TEST_F(RobotProfileTest, DefaultFramesAreMapOdomBaseFootprint)
{
  const auto node = make_node();

  const ConversionResult<RobotProfile> result = declare_robot_profile(*node);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().frames().map, "map");
  EXPECT_EQ(result.value().frames().odom, "odom");
  EXPECT_EQ(result.value().frames().base, "base_footprint");
}

TEST_F(RobotProfileTest, OverridesAreReadUnderTheDeclaredKeys)
{
  const auto node = make_node(
    {rclcpp::Parameter("robot.inflation_radius", 0.80),
     rclcpp::Parameter("robot.max_linear_vel", 0.12),
     rclcpp::Parameter("frames.base", std::string("base_link"))});

  const ConversionResult<RobotProfile> result = declare_robot_profile(*node);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NEAR(result.value().radii().inflation_radius(), 0.80, 1e-9);
  EXPECT_NEAR(result.value().limits().max_linear_vel, 0.12, 1e-9);
  EXPECT_EQ(result.value().frames().base, "base_link");
}

TEST_F(RobotProfileTest, DeclaringTwiceOnTheSameNodeIsAccepted)
{
  const auto node = make_node();
  ASSERT_TRUE(declare_robot_profile(*node).ok());

  const ConversionResult<RobotProfile> second = declare_robot_profile(*node);

  EXPECT_TRUE(second.ok()) << second.error();
}

TEST_F(RobotProfileTest, IntegerArrayFootprintIsRejectedNotThrown)
{
  const auto node =
    make_node({rclcpp::Parameter("robot.footprint", std::vector<std::int64_t>{0, 0, 1, 0, 1, 1})});

  ConversionResult<RobotProfile> result = ConversionResult<RobotProfile>::failure("not run");
  EXPECT_NO_THROW(result = declare_robot_profile(*node));

  ASSERT_FALSE(result.ok());
  EXPECT_TRUE(contains(result.error(), "robot.footprint"));
  EXPECT_TRUE(is_one_line(result.error()));
  EXPECT_TRUE(names_the_package_once(result.error()));
}

TEST_F(RobotProfileTest, OddFootprintLengthIsRejected)
{
  const auto node =
    make_node({rclcpp::Parameter("robot.footprint", std::vector<double>{0.0, 0.0, 1.0, 0.0, 1.0})});

  expect_rejected(declare_robot_profile(*node), "robot.footprint", "5");
}

TEST_F(RobotProfileTest, TooFewVerticesIsRejected)
{
  const auto node =
    make_node({rclcpp::Parameter("robot.footprint", std::vector<double>{-1.0, -1.0, 1.0, 1.0})});

  expect_rejected(declare_robot_profile(*node), "robot.footprint", "4");
}

TEST_F(RobotProfileTest, NonFiniteVertexIsRejected)
{
  std::vector<double> footprint = kachaka_footprint();
  footprint[3] = NOT_A_NUMBER;
  const auto node = make_node({rclcpp::Parameter("robot.footprint", footprint)});

  expect_rejected(declare_robot_profile(*node), "robot.footprint", "nan");
}

TEST_F(RobotProfileTest, DegenerateFootprintIsRejected)
{
  const auto node = make_node(
    {rclcpp::Parameter("robot.footprint", std::vector<double>{0.0, 0.0, 1.0, 0.0, 2.0, 0.0})});

  expect_rejected(declare_robot_profile(*node), "robot.footprint", "degenerate");
}

TEST_F(RobotProfileTest, NonConvexFootprintIsRejected)
{
  const auto node = make_node({rclcpp::Parameter(
    "robot.footprint", std::vector<double>{-1.0, -1.0, 0.0, -0.2, 1.0, -1.0, 0.0, 1.0})});

  expect_rejected(declare_robot_profile(*node), "robot.footprint", "convex");
}

TEST_F(RobotProfileTest, OriginOutsideFootprintIsRejected)
{
  const auto node = make_node({rclcpp::Parameter(
    "robot.footprint", std::vector<double>{0.5, 0.5, 1.0, 0.5, 1.0, 1.0, 0.5, 1.0})});

  expect_rejected(declare_robot_profile(*node), "robot.footprint", "origin");
}

TEST_F(RobotProfileTest, InflationBelowCircumscribedIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.inflation_radius", 0.20)});

  expect_rejected(declare_robot_profile(*node), "robot.inflation_radius", "0.20");
}

TEST_F(RobotProfileTest, NonFiniteInflationIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.inflation_radius", NOT_A_NUMBER)});

  expect_rejected(declare_robot_profile(*node), "robot.inflation_radius", "nan");
}

TEST_F(RobotProfileTest, NegativeCostScalingIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.cost_scaling_factor", -1.0)});

  expect_rejected(declare_robot_profile(*node), "robot.cost_scaling_factor", "-1.0");
}

TEST_F(RobotProfileTest, ZeroCostScalingIsAccepted)
{
  const auto node = make_node({rclcpp::Parameter("robot.cost_scaling_factor", 0.0)});

  EXPECT_TRUE(declare_robot_profile(*node).ok());
}

TEST_F(RobotProfileTest, ZeroMaxLinearVelIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.max_linear_vel", 0.0)});

  expect_rejected(declare_robot_profile(*node), "robot.max_linear_vel", "0.0");
}

TEST_F(RobotProfileTest, NegativeMaxAngularVelIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.max_angular_vel", -1.57)});

  expect_rejected(declare_robot_profile(*node), "robot.max_angular_vel", "-1.57");
}

TEST_F(RobotProfileTest, ZeroMaxAccelIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.max_accel", 0.0)});

  expect_rejected(declare_robot_profile(*node), "robot.max_accel", "0.0");
}

TEST_F(RobotProfileTest, ZeroMaxDecelIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("robot.max_decel", 0.0)});

  expect_rejected(declare_robot_profile(*node), "robot.max_decel", "0.0");
}

TEST_F(RobotProfileTest, EmptyFrameIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("frames.map", std::string(""))});

  expect_rejected(declare_robot_profile(*node), "frames.map", "tf2");
}

TEST_F(RobotProfileTest, FrameStartingWithSlashIsRejected)
{
  const auto node = make_node({rclcpp::Parameter("frames.odom", std::string("/odom"))});

  expect_rejected(declare_robot_profile(*node), "frames.odom", "/odom");
}

TEST_F(RobotProfileTest, ProfileCarriesTheInflationModel)
{
  const auto node = make_node();

  const ConversionResult<RobotProfile> result = declare_robot_profile(*node);

  ASSERT_TRUE(result.ok()) << result.error();
  const eltanin::map::InflationCostModel & model = result.value().inflation_cost_model();
  EXPECT_EQ(model.cost_at_distance(0.0), eltanin::map::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(model.cost_at_distance(1.0), eltanin::map::FREE_SPACE);
  EXPECT_GE(model.circumscribed_cost(), 1);
  EXPECT_EQ(model.radii().inscribed_radius(), result.value().radii().inscribed_radius());
}

TEST_F(RobotProfileTest, FootprintIsNormalizedToCounterClockwise)
{
  const auto node = make_node({rclcpp::Parameter(
    "robot.footprint",
    std::vector<double>{-0.150, 0.120, 0.237, 0.120, 0.237, -0.120, -0.150, -0.120})});

  const ConversionResult<RobotProfile> result = declare_robot_profile(*node);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_GT(eltanin::signed_area(result.value().footprint()), 0.0);
}

TEST_F(RobotProfileTest, ExactFootprintCheckIsNotAParameter)
{
  const auto node = make_node();
  ASSERT_TRUE(declare_robot_profile(*node).ok());

  EXPECT_FALSE(node->has_parameter("robot.exact_footprint_check"));
  EXPECT_TRUE(eltanin::collision::VelocityLimiterParams{}.exact_footprint_check);
}

}  // namespace
}  // namespace eltanin_ros_common::test
