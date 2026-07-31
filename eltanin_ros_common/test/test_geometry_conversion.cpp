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

#include "eltanin_ros_common/geometry_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <eltanin/core/angle.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace
{

using eltanin_ros_common::PlanarityTolerance;
using eltanin_ros_common::QUATERNION_NORM_TOLERANCE;
using eltanin_ros_common::to_pose2d;
using eltanin_ros_common::to_pose_msg;
using eltanin_ros_common::to_quaternion;
using eltanin_ros_common::to_roll_pitch;
using eltanin_ros_common::to_transform2d;
using eltanin_ros_common::to_twist2d;
using eltanin_ros_common::to_twist_msg;
using eltanin_ros_common::to_yaw;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;

constexpr double PI = std::numbers::pi;
constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

geometry_msgs::msg::Quaternion make_quaternion(double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  msg.w = w;
  return msg;
}

TEST(ToYawTest, RoundTripsThroughAQuaternion)
{
  for (const double yaw : {0.0, 0.1, -0.1, PI / 2.0, -PI / 2.0, 2.0, -2.0, PI, -PI, 3.0 * PI}) {
    const auto result = to_yaw(to_quaternion(yaw));
    ASSERT_TRUE(result.ok()) << "yaw " << yaw << ": " << result.error();
    EXPECT_NEAR(result.value(), eltanin::normalize_angle(yaw), 1e-12) << "yaw " << yaw;
  }
}

TEST(ToYawTest, StaysInTheHalfOpenRange)
{
  for (const double yaw : {PI, -PI, 3.0 * PI, -3.0 * PI, 100.0, -100.0}) {
    const auto result = to_yaw(to_quaternion(yaw));
    ASSERT_TRUE(result.ok()) << "yaw " << yaw;
    EXPECT_GT(result.value(), -PI) << "yaw " << yaw;
    EXPECT_LE(result.value(), PI) << "yaw " << yaw;
  }
}

TEST(ToYawTest, NegativePiComesBackAsPositivePi)
{
  const auto result = to_yaw(to_quaternion(-PI));
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value(), PI, 1e-12);
}

TEST(ToYawTest, RejectsTheZeroQuaternionThatTf2WouldReadAsZeroYaw)
{
  const auto result = to_yaw(make_quaternion(0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains(result.error(), "norm"));
  EXPECT_TRUE(is_one_line(result.error()));
}

TEST(ToYawTest, ADefaultConstructedQuaternionIsTheIdentityAndIsAccepted)
{
  const geometry_msgs::msg::Quaternion identity;
  EXPECT_DOUBLE_EQ(identity.w, 1.0);
  const auto result = to_yaw(identity);
  ASSERT_TRUE(result.ok());
  EXPECT_DOUBLE_EQ(result.value(), 0.0);
}

TEST(ToYawTest, RejectsAnUnnormalizedQuaternion)
{
  EXPECT_FALSE(to_yaw(make_quaternion(0.0, 0.0, 0.0, 0.5)).ok());
  EXPECT_FALSE(to_yaw(make_quaternion(0.0, 0.0, 0.0, 2.0)).ok());
}

TEST(ToYawTest, AcceptsSmallNormalizationError)
{
  const double within = 1.0 + 0.5 * QUATERNION_NORM_TOLERANCE;
  EXPECT_TRUE(to_yaw(make_quaternion(0.0, 0.0, 0.0, within)).ok());
}

TEST(ToYawTest, RejectsNonFiniteComponents)
{
  const auto result = to_yaw(make_quaternion(0.0, 0.0, NOT_A_NUMBER, 1.0));
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains(result.error(), "finite"));
}

TEST(ToRollPitchTest, SeesTheRotationThatToYawReportsAsZero)
{
  const geometry_msgs::msg::Quaternion roll_only = make_quaternion(1.0, 0.0, 0.0, 0.0);
  const auto yaw = to_yaw(roll_only);
  ASSERT_TRUE(yaw.ok());
  EXPECT_NEAR(yaw.value(), 0.0, 1e-12);

  const auto roll_pitch = to_roll_pitch(roll_only);
  ASSERT_TRUE(roll_pitch.ok());
  EXPECT_NEAR(std::abs(roll_pitch.value().roll), PI, 1e-12);
  EXPECT_NEAR(roll_pitch.value().pitch, 0.0, 1e-12);
}

TEST(ToRollPitchTest, IsZeroForAPlanarRotation)
{
  const auto result = to_roll_pitch(to_quaternion(0.7));
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().roll, 0.0, 1e-12);
  EXPECT_NEAR(result.value().pitch, 0.0, 1e-12);
}

TEST(ToRollPitchTest, RejectsTheSameQuaternionsAsToYaw)
{
  EXPECT_FALSE(to_roll_pitch(make_quaternion(0.0, 0.0, 0.0, 0.0)).ok());
  EXPECT_FALSE(to_roll_pitch(make_quaternion(0.0, 0.0, 0.0, 0.5)).ok());
  EXPECT_FALSE(to_roll_pitch(make_quaternion(NOT_A_NUMBER, 0.0, 0.0, 1.0)).ok());
}

TEST(ToQuaternionTest, PutsTheRotationAboutZAndNotAboutX)
{
  const geometry_msgs::msg::Quaternion msg = to_quaternion(PI / 2.0);
  EXPECT_NEAR(msg.x, 0.0, 1e-12);
  EXPECT_NEAR(msg.y, 0.0, 1e-12);
  EXPECT_NEAR(msg.z, std::sin(PI / 4.0), 1e-12);
  EXPECT_NEAR(msg.w, std::cos(PI / 4.0), 1e-12);
}

TEST(PoseConversionTest, RoundTripsAndDropsTheThirdDimension)
{
  const eltanin::Pose2D pose{Eigen::Vector2d{1.25, -3.5}, 0.75};
  const geometry_msgs::msg::Pose msg = to_pose_msg(pose);
  EXPECT_DOUBLE_EQ(msg.position.z, 0.0);

  const auto back = to_pose2d(msg);
  ASSERT_TRUE(back.ok());
  EXPECT_DOUBLE_EQ(back.value().position.x(), pose.position.x());
  EXPECT_DOUBLE_EQ(back.value().position.y(), pose.position.y());
  EXPECT_NEAR(back.value().yaw, pose.yaw, 1e-12);
}

TEST(PoseConversionTest, IgnoresPositionZOnTheWayIn)
{
  geometry_msgs::msg::Pose msg = to_pose_msg(eltanin::Pose2D{Eigen::Vector2d{1.0, 2.0}, 0.0});
  msg.position.z = 9.0;
  const auto result = to_pose2d(msg);
  ASSERT_TRUE(result.ok());
  EXPECT_DOUBLE_EQ(result.value().position.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.value().position.y(), 2.0);
}

TEST(PoseConversionTest, RejectsNonFinitePositionAndBadOrientation)
{
  geometry_msgs::msg::Pose msg = to_pose_msg(eltanin::Pose2D{Eigen::Vector2d{1.0, 2.0}, 0.0});
  msg.position.x = NOT_A_NUMBER;
  const auto position = to_pose2d(msg);
  EXPECT_FALSE(position.ok());
  EXPECT_TRUE(contains(position.error(), "finite"));

  geometry_msgs::msg::Pose zero_orientation;
  zero_orientation.orientation = make_quaternion(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(to_pose2d(zero_orientation).ok());
}

TEST(TwistConversionTest, DropsEveryComponentADifferentialDriveCannotExecute)
{
  const eltanin::Twist2D twist{Eigen::Vector2d{0.4, 0.9}, -0.2};
  const geometry_msgs::msg::Twist msg = to_twist_msg(twist);
  EXPECT_DOUBLE_EQ(msg.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(msg.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular.z, -0.2);
}

TEST(TwistConversionTest, ReadsOnlyLinearXAndAngularZ)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.3;
  msg.linear.y = 1.0;
  msg.linear.z = 2.0;
  msg.angular.x = 3.0;
  msg.angular.y = 4.0;
  msg.angular.z = 0.1;
  const auto result = to_twist2d(msg);
  ASSERT_TRUE(result.ok());
  EXPECT_DOUBLE_EQ(result.value().linear.x(), 0.3);
  EXPECT_DOUBLE_EQ(result.value().linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.value().angular, 0.1);
}

TEST(TwistConversionTest, DoesNotRejectNonFiniteValuesInDiscardedFields)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.3;
  msg.linear.y = NOT_A_NUMBER;
  msg.angular.x = NOT_A_NUMBER;
  EXPECT_TRUE(to_twist2d(msg).ok());
}

TEST(TwistConversionTest, RejectsNonFiniteValuesInTheFieldsItReads)
{
  geometry_msgs::msg::Twist linear;
  linear.linear.x = NOT_A_NUMBER;
  EXPECT_FALSE(to_twist2d(linear).ok());

  geometry_msgs::msg::Twist angular;
  angular.angular.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(to_twist2d(angular).ok());
}

geometry_msgs::msg::TransformStamped make_transform(
  double x, double y, double z, const geometry_msgs::msg::Quaternion & rotation)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = "map";
  msg.child_frame_id = "base_link";
  msg.transform.translation.x = x;
  msg.transform.translation.y = y;
  msg.transform.translation.z = z;
  msg.transform.rotation = rotation;
  return msg;
}

TEST(ToTransform2dTest, TakesTranslationXYAndYaw)
{
  const auto result = make_transform(2.0, -1.0, 0.0, to_quaternion(0.5));
  const auto converted = to_transform2d(result);
  ASSERT_TRUE(converted.ok());
  EXPECT_DOUBLE_EQ(converted.value().transform.translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(converted.value().transform.translation().y(), -1.0);
  EXPECT_NEAR(converted.value().transform.rotation(), 0.5, 1e-12);
  EXPECT_TRUE(converted.value().deviation.within_tolerance);
  EXPECT_DOUBLE_EQ(converted.value().deviation.z, 0.0);
}

TEST(ToTransform2dTest, ReportsAnOutOfPlaneTransformWithoutRejectingIt)
{
  tf2::Quaternion tilted;
  tilted.setRPY(0.2, 0.0, 0.5);
  const auto converted = to_transform2d(
    make_transform(0.0, 0.0, 0.3, make_quaternion(tilted.x(), tilted.y(), tilted.z(), tilted.w())));
  ASSERT_TRUE(converted.ok());
  EXPECT_FALSE(converted.value().deviation.within_tolerance);
  EXPECT_NEAR(converted.value().deviation.z, 0.3, 1e-12);
  EXPECT_NEAR(converted.value().deviation.roll, 0.2, 1e-12);
  EXPECT_NEAR(converted.value().deviation.pitch, 0.0, 1e-12);
  EXPECT_NEAR(converted.value().transform.rotation(), 0.5, 1e-12);
}

TEST(ToTransform2dTest, HonoursTheToleranceArgument)
{
  const auto msg = make_transform(0.0, 0.0, 0.3, to_quaternion(0.0));
  EXPECT_FALSE(to_transform2d(msg).value().deviation.within_tolerance);
  EXPECT_TRUE(
    to_transform2d(msg, PlanarityTolerance{0.5, 0.05, 0.05}).value().deviation.within_tolerance);
}

TEST(ToTransform2dTest, RejectsNonFiniteTranslationAndBadRotation)
{
  const auto translation =
    to_transform2d(make_transform(NOT_A_NUMBER, 0.0, 0.0, to_quaternion(0.0)));
  EXPECT_FALSE(translation.ok());
  EXPECT_TRUE(contains(translation.error(), "finite"));
  EXPECT_TRUE(contains(translation.error(), "base_link"));

  const auto height = to_transform2d(make_transform(0.0, 0.0, NOT_A_NUMBER, to_quaternion(0.0)));
  EXPECT_FALSE(height.ok());

  const auto rotation =
    to_transform2d(make_transform(0.0, 0.0, 0.0, make_quaternion(0.0, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(rotation.ok());
}

}  // namespace
