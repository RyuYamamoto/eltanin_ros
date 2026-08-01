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

#include "eltanin_planner/global_path_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using eltanin_planner::GlobalPathPlanner;

/// Long enough that a loaded CI machine still gets there, short enough to fail rather than hang.
constexpr std::chrono::seconds DEADLINE{10};
constexpr std::chrono::milliseconds POLL_INTERVAL{2};

/// Both nodes share one MultiThreadedExecutor, the executor the generated entry point uses.
class GlobalPathPlannerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(helper_);
    spinner_ = std::thread([this]() { executor_->spin(); });
    // cancel() before spin() has begun is ignored, and the join in TearDown would never return.
    while (!executor_->is_spinning()) {
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
  }

  void TearDown() override
  {
    executor_->cancel();
    spinner_.join();
    node_.reset();
    helper_.reset();
    executor_.reset();
  }

  /// Starts the node under test; a construction failure is left to the caller to observe.
  void start(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    node_ = std::make_shared<GlobalPathPlanner>(options);
    executor_->add_node(node_);
  }

  /// Waits on a predicate rather than for a guessed duration, and says what it was waiting for.
  template <class Predicate>
  ::testing::AssertionResult wait_until(const std::string & what, Predicate predicate)
  {
    const auto expiry = std::chrono::steady_clock::now() + DEADLINE;
    while (std::chrono::steady_clock::now() < expiry) {
      if (predicate()) {
        return ::testing::AssertionSuccess();
      }
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
    return ::testing::AssertionFailure() << "timed out waiting until " << what;
  }

  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<GlobalPathPlanner> node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spinner_;
};

TEST_F(GlobalPathPlannerFixture, StartsWithNoParametersAtAll)
{
  ASSERT_NO_THROW(start());
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(GlobalPathPlannerFixture, RefusesToStartOnSmootherWeightsThatDiverge)
{
  try {
    start({rclcpp::Parameter("weight_data", 0.5), rclcpp::Parameter("weight_smooth", 0.4)});
    FAIL() << "the node started with smoother weights that diverge";
  } catch (const std::runtime_error & error) {
    const std::string line = error.what();
    EXPECT_NE(line.find("weight_smooth"), std::string::npos) << line;
    EXPECT_NE(line.find("diverges"), std::string::npos) << line;
    EXPECT_EQ(line.find('\n'), std::string::npos) << line;
  }
}

TEST_F(GlobalPathPlannerFixture, RefusesToStartOnANegativeStartSearchRadius)
{
  EXPECT_THROW(start({rclcpp::Parameter("start_search_radius_cells", -1)}), std::runtime_error);
}

TEST_F(GlobalPathPlannerFixture, RefusesToStartOnANegativeTfLookupTimeout)
{
  EXPECT_THROW(start({rclcpp::Parameter("tf_lookup_timeout", -0.1)}), std::runtime_error);
}

TEST_F(GlobalPathPlannerFixture, RefusesToStartOnAFootprintThatIsNotAShape)
{
  EXPECT_THROW(
    start({rclcpp::Parameter("robot.footprint", std::vector<double>{0.0, 0.0})}),
    std::runtime_error);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
