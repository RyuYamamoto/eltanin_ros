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

#include "eltanin_ros_common/conversion_result.hpp"
#include "eltanin_ros_common/warn_once.hpp"

#include <gtest/gtest.h>

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{

using eltanin_ros_common::ConversionResult;
using eltanin_ros_common::ConversionStatus;
using eltanin_ros_common::WarnOnceLatch;

TEST(ConversionStatusTest, SuccessHasNoMessage)
{
  const ConversionStatus status = ConversionStatus::success();
  EXPECT_TRUE(status.ok());
  EXPECT_TRUE(static_cast<bool>(status));
  EXPECT_TRUE(status.message().empty());
}

TEST(ConversionStatusTest, FailureKeepsTheMessage)
{
  const ConversionStatus status = ConversionStatus::failure("resolution is 0");
  EXPECT_FALSE(status.ok());
  EXPECT_FALSE(static_cast<bool>(status));
  EXPECT_EQ(status.message(), "resolution is 0");
}

TEST(ConversionResultTest, SuccessCarriesTheValueAndNoError)
{
  const ConversionResult<int> result = ConversionResult<int>::success(42);
  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.value(), 42);
  EXPECT_TRUE(result.error().empty());
}

TEST(ConversionResultTest, FailureCarriesTheErrorAndNoValue)
{
  const ConversionResult<int> result = ConversionResult<int>::failure("width is 0");
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(static_cast<bool>(result));
  EXPECT_EQ(result.error(), "width is 0");
}

TEST(ConversionResultTest, ValueIsMovableOut)
{
  ConversionResult<std::vector<int>> result =
    ConversionResult<std::vector<int>>::success(std::vector<int>{1, 2, 3});
  const std::vector<int> taken = std::move(result.value());
  EXPECT_EQ(taken, (std::vector<int>{1, 2, 3}));
}

TEST(ConversionResultTest, DoesNotConvertImplicitlyToBool)
{
  static_assert(!std::is_convertible_v<ConversionResult<int>, bool>);
  static_assert(!std::is_convertible_v<ConversionStatus, bool>);
}

TEST(WarnOnceLatchTest, WarnsOnceAndThenStaysQuiet)
{
  WarnOnceLatch latch;
  EXPECT_TRUE(latch.should_warn());
  EXPECT_FALSE(latch.should_warn());
  EXPECT_FALSE(latch.should_warn());
}

TEST(WarnOnceLatchTest, LatchesAreIndependent)
{
  WarnOnceLatch first;
  WarnOnceLatch second;
  EXPECT_TRUE(first.should_warn());
  EXPECT_TRUE(second.should_warn());
  EXPECT_FALSE(first.should_warn());
  EXPECT_FALSE(second.should_warn());
}

}  // namespace
