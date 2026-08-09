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

#include "eltanin_controller/path_input.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <optional>

namespace
{

using eltanin_controller::PathInput;
using eltanin_controller::PathSnapshot;
using State = eltanin_controller::PathInput::State;

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

PathSnapshot make_snapshot(double stamp_seconds)
{
  PathSnapshot snapshot;
  snapshot.path = std::make_shared<const eltanin::Path>(eltanin::Path{
    eltanin::Pose2D{Eigen::Vector2d{0.0, 0.0}, 0.0},
    eltanin::Pose2D{Eigen::Vector2d{1.0, 0.0}, 0.0}});
  snapshot.stamp = at(stamp_seconds);
  return snapshot;
}

TEST(PathInputTest, NothingReceivedIsNotARejection)
{
  const PathInput input{std::nullopt};
  const PathInput::Reading reading = input.read(at(100.0));
  EXPECT_EQ(reading.state, State::NeverReceived);
  EXPECT_EQ(reading.snapshot.path, nullptr);
}

TEST(PathInputTest, ARejectionBeforeAnyPathIsItsOwnState)
{
  PathInput input{std::nullopt};
  input.reject("frame_id is 'odom'");
  const PathInput::Reading reading = input.read(at(100.0));
  EXPECT_EQ(reading.state, State::Rejected);
  EXPECT_EQ(input.rejection_detail(), "frame_id is 'odom'");
}

TEST(PathInputTest, WithoutADeadlineAPathStaysAvailableForever)
{
  PathInput input{std::nullopt};
  input.accept(make_snapshot(10.0));

  const PathInput::Reading reading = input.read(at(1000000.0));
  EXPECT_EQ(reading.state, State::Available);
  ASSERT_NE(reading.snapshot.path, nullptr);
  EXPECT_EQ(reading.snapshot.path->size(), 2u);
  EXPECT_NEAR(reading.elapsed_seconds, 999990.0, 1e-3);
}

TEST(PathInputTest, WithADeadlineTheEdgeIsInclusiveAndBeyondItIsStale)
{
  PathInput input{0.5};
  input.accept(make_snapshot(10.0));

  EXPECT_EQ(input.read(at(10.5)).state, State::Available);
  EXPECT_EQ(input.read(at(10.6)).state, State::Stale);
}

TEST(PathInputTest, AStampFromTheFutureIsStaleToo)
{
  PathInput input{0.5};
  input.accept(make_snapshot(10.0));

  const PathInput::Reading reading = input.read(at(9.0));
  EXPECT_EQ(reading.state, State::Stale);
  EXPECT_NEAR(reading.elapsed_seconds, -1.0, 1e-6);
}

TEST(PathInputTest, AStaleReadingStillCarriesThePathItIsAbout)
{
  PathInput input{0.5};
  input.accept(make_snapshot(10.0));

  const PathInput::Reading reading = input.read(at(20.0));
  ASSERT_EQ(reading.state, State::Stale);
  EXPECT_NE(reading.snapshot.path, nullptr);
}

TEST(PathInputTest, ARejectionDoesNotThrowAwayThePathAlreadyBeingFollowed)
{
  PathInput input{std::nullopt};
  input.accept(make_snapshot(10.0));
  input.reject("frame_id is 'odom'");

  const PathInput::Reading reading = input.read(at(11.0));
  EXPECT_EQ(reading.state, State::Available);
  EXPECT_EQ(input.rejection_detail(), "frame_id is 'odom'");
}

TEST(PathInputTest, AcceptingClearsTheRejectionRecord)
{
  PathInput input{std::nullopt};
  input.reject("frame_id is 'odom'");
  input.accept(make_snapshot(10.0));

  EXPECT_EQ(input.read(at(11.0)).state, State::Available);
  EXPECT_TRUE(input.rejection_detail().empty());
}

TEST(PathInputTest, ClearingForgetsThePathAndTheRejection)
{
  PathInput input{std::nullopt};
  input.accept(make_snapshot(10.0));
  input.reject("frame_id is 'odom'");

  input.clear();

  const PathInput::Reading reading = input.read(at(11.0));
  EXPECT_EQ(reading.state, State::NeverReceived);
  EXPECT_EQ(reading.snapshot.path, nullptr);
  EXPECT_TRUE(input.rejection_detail().empty());
}

TEST(PathInputTest, AnEmptyPathIsAvailableAndTheNodeDecidesWhatThatMeans)
{
  PathInput input{std::nullopt};
  PathSnapshot snapshot;
  snapshot.path = std::make_shared<const eltanin::Path>();
  snapshot.stamp = at(10.0);
  input.accept(std::move(snapshot));

  const PathInput::Reading reading = input.read(at(11.0));
  EXPECT_EQ(reading.state, State::Available);
  EXPECT_TRUE(reading.snapshot.path->empty());
}

}  // namespace
