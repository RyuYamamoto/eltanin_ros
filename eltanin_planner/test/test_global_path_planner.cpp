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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin_ros_common/geometry_conversion.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <eltanin_msgs/action/compute_path_to_pose.hpp>
#include <eltanin_msgs/msg/costmap.hpp>
#include <eltanin_msgs/msg/costmap_update.hpp>
#include <eltanin_msgs/msg/navigation_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin_msgs::msg::NavigationState;
using eltanin_planner::GlobalPathPlanner;
using Action = eltanin_msgs::action::ComputePathToPose;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;

/// Long enough that a loaded CI machine still gets there, short enough to fail rather than hang.
constexpr std::chrono::seconds DEADLINE{10};
constexpr std::chrono::milliseconds POLL_INTERVAL{2};

constexpr double RESOLUTION = 0.1;
constexpr std::uint32_t WIDTH = 8;
constexpr std::uint32_t HEIGHT = 8;

/// The lookup a goal without tf blocks on; long enough to hold the worker for a whole test step.
constexpr double BLOCKING_TF_TIMEOUT = 2.0;

double cell_center(int index)
{
  return (static_cast<double>(index) + 0.5) * RESOLUTION;
}

eltanin_msgs::msg::Costmap make_costmap_msg(std::int32_t sec, const std::string & frame_id = "map")
{
  eltanin_msgs::msg::Costmap msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.sec = sec;
  msg.info.resolution = RESOLUTION;
  msg.info.width = WIDTH;
  msg.info.height = HEIGHT;
  msg.info.origin_x = 0.0;
  msg.info.origin_y = 0.0;
  msg.data.assign(static_cast<std::size_t>(WIDTH) * HEIGHT, FREE_SPACE);
  return msg;
}

void block_cell(eltanin_msgs::msg::Costmap & msg, std::uint32_t mx, std::uint32_t my)
{
  msg.data[my * WIDTH + mx] = LETHAL_OBSTACLE;
}

eltanin_msgs::msg::CostmapUpdate make_patch(
  std::int32_t sec, std::uint32_t mx, std::uint32_t my, std::uint8_t value)
{
  eltanin_msgs::msg::CostmapUpdate msg;
  msg.header.frame_id = "map";
  msg.header.stamp.sec = sec;
  msg.x = mx;
  msg.y = my;
  msg.width = 1;
  msg.height = 1;
  msg.data = {value};
  return msg;
}

geometry_msgs::msg::PoseStamped make_pose(
  const std::string & frame_id, double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.orientation = eltanin_ros_common::to_quaternion(yaw);
  return msg;
}

/// A goal that can be planned: cell (1, 1) to cell (6, 6) of the empty 8x8 map.
Action::Goal make_goal(double goal_yaw = 0.0)
{
  Action::Goal goal;
  goal.goal = make_pose("map", cell_center(6), cell_center(6), goal_yaw);
  goal.start = make_pose("map", cell_center(1), cell_center(1));
  goal.use_start = true;
  return goal;
}

::testing::AssertionResult is_one_line(const std::string & text)
{
  if (text.empty()) {
    return ::testing::AssertionFailure() << "the message is empty";
  }
  if (text.find('\n') != std::string::npos) {
    return ::testing::AssertionFailure() << "'" << text << "' contains a newline";
  }
  return ::testing::AssertionSuccess();
}

/// The shipped configuration, so a key missing from it fails here instead of on the robot.
std::vector<rclcpp::Parameter> shipped_configuration()
{
  std::vector<rclcpp::Parameter> parameters;
  const std::vector<std::string> files{
    ament_index_cpp::get_package_share_directory("eltanin_ros_common") +
      "/config/robot/kachaka.yaml",
    ament_index_cpp::get_package_share_directory("eltanin_planner") +
      "/config/global_path_planner.param.yaml"};
  for (const std::string & file : files) {
    for (const auto & [node_name, values] : rclcpp::parameter_map_from_yaml_file(file)) {
      (void)node_name;
      parameters.insert(parameters.end(), values.begin(), values.end());
    }
  }
  return parameters;
}

/// Both nodes share one MultiThreadedExecutor, the executor the generated entry point uses.
class GlobalPathPlannerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    costmap_publisher_ = helper_->create_publisher<eltanin_msgs::msg::Costmap>(
      "/global_costmap/global_costmap",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    patch_publisher_ = helper_->create_publisher<eltanin_msgs::msg::CostmapUpdate>(
      "/global_costmap/global_costmap_updates", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    path_subscription_ = helper_->create_subscription<nav_msgs::msg::Path>(
      "/global_path_planner/global_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { last_path_ = msg; });
    raw_path_subscription_ = helper_->create_subscription<nav_msgs::msg::Path>(
      "/global_path_planner/global_path_raw", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { last_raw_path_ = msg; });

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
    client_.reset();
    node_.reset();
    helper_.reset();
    executor_.reset();
  }

  /// Starts the node under test; a construction failure is left to the caller to observe.
  void start(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    std::vector<rclcpp::Parameter> merged = shipped_configuration();
    merged.insert(merged.end(), parameters.begin(), parameters.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(merged);
    node_ = std::make_shared<GlobalPathPlanner>(options);
    executor_->add_node(node_);
    client_ =
      rclcpp_action::create_client<Action>(helper_, "/global_path_planner/compute_path_to_pose");
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

  /// Sends a goal and returns its client handle, or nullptr when the server never accepted it.
  ClientGoalHandle::SharedPtr send(const Action::Goal & goal)
  {
    if (!client_->wait_for_action_server(DEADLINE)) {
      return nullptr;
    }
    auto accepted = client_->async_send_goal(goal);
    if (accepted.wait_for(DEADLINE) != std::future_status::ready) {
      return nullptr;
    }
    return accepted.get();
  }

  Action::Result::SharedPtr wait_for_result(
    const ClientGoalHandle::SharedPtr & handle, rclcpp_action::ResultCode * code = nullptr)
  {
    auto future = client_->async_get_result(handle);
    if (future.wait_for(DEADLINE) != std::future_status::ready) {
      return nullptr;
    }
    const ClientGoalHandle::WrappedResult wrapped = future.get();
    if (code != nullptr) {
      *code = wrapped.code;
    }
    return wrapped.result;
  }

  Action::Result::SharedPtr plan(
    const Action::Goal & goal, rclcpp_action::ResultCode * code = nullptr)
  {
    const ClientGoalHandle::SharedPtr handle = send(goal);
    if (!handle) {
      return nullptr;
    }
    return wait_for_result(handle, code);
  }

  /// The node signals nothing when a costmap lands, so the readiness probe is a plan request.
  ::testing::AssertionResult publish_costmap(const eltanin_msgs::msg::Costmap & msg)
  {
    const auto matched = wait_until("the node subscribes to the costmap", [this]() {
      return costmap_publisher_->get_subscription_count() > 0;
    });
    if (!matched) {
      return matched;
    }
    costmap_publisher_->publish(msg);
    return wait_until("the costmap has been taken up", [this]() {
      const auto result = plan(make_goal());
      return result != nullptr && result->outcome != NavigationState::OUTCOME_INPUT_STALE;
    });
  }

  ::testing::AssertionResult publish_patch(const eltanin_msgs::msg::CostmapUpdate & msg)
  {
    const auto matched = wait_until("the node subscribes to the patches", [this]() {
      return patch_publisher_->get_subscription_count() > 0;
    });
    if (!matched) {
      return matched;
    }
    patch_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  void broadcast(const std::string & parent, const std::string & child, double x, double y)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(transform);
  }

  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<GlobalPathPlanner> node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spinner_;
  rclcpp::Publisher<eltanin_msgs::msg::Costmap>::SharedPtr costmap_publisher_;
  rclcpp::Publisher<eltanin_msgs::msg::CostmapUpdate>::SharedPtr patch_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_subscription_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  nav_msgs::msg::Path::ConstSharedPtr last_path_;
  nav_msgs::msg::Path::ConstSharedPtr last_raw_path_;
};

TEST_F(GlobalPathPlannerFixture, StartsWithNoParametersAtAll)
{
  ASSERT_NO_THROW(start());
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(GlobalPathPlannerFixture, StartsUnderSimulatedTime)
{
  ASSERT_NO_THROW(start({rclcpp::Parameter("use_sim_time", true)}));
  EXPECT_TRUE(node_->get_clock()->ros_time_is_active());
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
    EXPECT_TRUE(is_one_line(line));
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

TEST_F(GlobalPathPlannerFixture, AGoalBeforeAnyCostmapIsInputStale)
{
  start();
  const auto result = plan(make_goal());
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_INPUT_STALE);
  EXPECT_TRUE(is_one_line(result->message));
  EXPECT_TRUE(result->path.poses.empty());
}

TEST_F(GlobalPathPlannerFixture, AnEmptyGoalFrameIsRejectedRatherThanAssumedToBeMap)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal goal = make_goal();
  goal.goal.header.frame_id.clear();
  const auto result = plan(goal);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_NE(result->message.find("frame_id"), std::string::npos) << result->message;
  EXPECT_TRUE(is_one_line(result->message));
}

TEST_F(GlobalPathPlannerFixture, AGoalInAFrameWithNoTransformIsRejected)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal goal = make_goal();
  goal.goal = make_pose("nowhere", 0.0, 0.0);
  const auto result = plan(goal);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_NE(result->message.find("nowhere"), std::string::npos) << result->message;
  EXPECT_TRUE(is_one_line(result->message));
}

TEST_F(GlobalPathPlannerFixture, AGoalInAnotherFrameIsTransformedIntoMap)
{
  start();
  broadcast("map", "other", cell_center(6), cell_center(6));
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal goal = make_goal();
  goal.goal = make_pose("other", 0.0, 0.0);
  const auto result = plan(goal);
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->outcome, NavigationState::OUTCOME_REACHED) << result->message;
  ASSERT_FALSE(result->path.poses.empty());

  const auto & last = result->path.poses.back().pose.position;
  EXPECT_NEAR(last.x, cell_center(6), RESOLUTION);
  EXPECT_NEAR(last.y, cell_center(6), RESOLUTION);
  EXPECT_EQ(result->path.header.frame_id, "map");
}

TEST_F(GlobalPathPlannerFixture, ASuccessfulPlanPublishesThePathItReturnsAndKeepsTheGoalYaw)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  last_path_ = nullptr;

  const double goal_yaw = 1.2345;
  const auto result = plan(make_goal(goal_yaw));
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->outcome, NavigationState::OUTCOME_REACHED) << result->message;
  EXPECT_TRUE(is_one_line(result->message));

  ASSERT_TRUE(wait_until("the path is published", [this]() { return last_path_ != nullptr; }));
  EXPECT_EQ(last_path_->poses.size(), result->path.poses.size());
  EXPECT_EQ(last_path_->header.frame_id, "map");

  ASSERT_FALSE(result->path.poses.empty());
  const auto yaw = eltanin_ros_common::to_yaw(result->path.poses.back().pose.orientation);
  ASSERT_TRUE(yaw.ok()) << yaw.error();
  EXPECT_NEAR(yaw.value(), goal_yaw, 1e-9);
}

TEST_F(GlobalPathPlannerFixture, NoRawPathPublisherExistsUnlessItIsAskedFor)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  ASSERT_EQ(plan(make_goal())->outcome, NavigationState::OUTCOME_REACHED);
  EXPECT_EQ(helper_->count_publishers("/global_path_planner/global_path_raw"), 0u);
}

TEST_F(GlobalPathPlannerFixture, TheUnsmoothedPathIsPublishedWhenItIsAskedFor)
{
  start({rclcpp::Parameter("publish_raw_path", true)});
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  last_raw_path_ = nullptr;

  ASSERT_EQ(plan(make_goal())->outcome, NavigationState::OUTCOME_REACHED);
  ASSERT_TRUE(
    wait_until("the raw path is published", [this]() { return last_raw_path_ != nullptr; }));
  EXPECT_FALSE(last_raw_path_->poses.empty());
  EXPECT_EQ(last_raw_path_->header.frame_id, "map");
}

TEST_F(GlobalPathPlannerFixture, AFailedPlanPublishesNothing)
{
  start();
  eltanin_msgs::msg::Costmap msg = make_costmap_msg(1);
  block_cell(msg, 6, 6);
  ASSERT_TRUE(wait_until("the node subscribes to the costmap", [this]() {
    return costmap_publisher_->get_subscription_count() > 0;
  }));
  costmap_publisher_->publish(msg);

  rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
  ASSERT_TRUE(wait_until("the blocked costmap has been taken up", [&]() {
    const auto result = plan(make_goal(), &code);
    return result != nullptr && result->outcome == NavigationState::OUTCOME_NO_PATH;
  }));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(last_path_, nullptr);
}

TEST_F(GlobalPathPlannerFixture, AStartOutsideTheMapIsAStartGoalFailure)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal goal = make_goal();
  goal.start = make_pose("map", -5.0, -5.0);
  const auto result = plan(goal);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_TRUE(is_one_line(result->message));
}

TEST_F(GlobalPathPlannerFixture, WithoutUseStartThePlanBeginsAtTheTransformedRobotPose)
{
  start();
  broadcast("map", "base_footprint", cell_center(5), cell_center(5));
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal goal = make_goal();
  goal.use_start = false;
  const auto result = plan(goal);
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->outcome, NavigationState::OUTCOME_REACHED) << result->message;
  ASSERT_FALSE(result->path.poses.empty());
  EXPECT_NEAR(result->path.poses.front().pose.position.x, cell_center(5), RESOLUTION);
  EXPECT_NEAR(result->path.poses.front().pose.position.y, cell_center(5), RESOLUTION);

  goal.use_start = true;
  const auto from_start = plan(goal);
  ASSERT_NE(from_start, nullptr);
  ASSERT_EQ(from_start->outcome, NavigationState::OUTCOME_REACHED) << from_start->message;
  EXPECT_NEAR(from_start->path.poses.front().pose.position.x, cell_center(1), RESOLUTION);
}

TEST_F(GlobalPathPlannerFixture, WithoutUseStartAndWithoutTfThePlanIsAStartGoalFailure)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal goal = make_goal();
  goal.use_start = false;
  const auto result = plan(goal);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_START_GOAL_FAILED);
  EXPECT_NE(result->message.find("base_footprint"), std::string::npos) << result->message;
  EXPECT_TRUE(is_one_line(result->message));
}

TEST_F(GlobalPathPlannerFixture, APatchClosesTheOnlyCorridorTheNextPlanCouldUse)
{
  start();
  eltanin_msgs::msg::Costmap msg = make_costmap_msg(1);
  for (std::uint32_t my = 1; my < HEIGHT; ++my) {
    block_cell(msg, 4, my);
  }
  ASSERT_TRUE(wait_until("the node subscribes to the costmap", [this]() {
    return costmap_publisher_->get_subscription_count() > 0;
  }));
  costmap_publisher_->publish(msg);
  ASSERT_TRUE(wait_until("the walled costmap has been taken up", [this]() {
    const auto result = plan(make_goal());
    return result != nullptr && result->outcome == NavigationState::OUTCOME_REACHED;
  }));

  ASSERT_TRUE(publish_patch(make_patch(2, 4, 0, LETHAL_OBSTACLE)));
  ASSERT_TRUE(wait_until("the patch closes the corridor", [this]() {
    const auto result = plan(make_goal());
    return result != nullptr && result->outcome == NavigationState::OUTCOME_PLAN_FAILED;
  }));
}

TEST_F(GlobalPathPlannerFixture, APatchThatIsNotNewerThanTheCostmapIsNotAppliedTwice)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  ASSERT_TRUE(publish_patch(make_patch(1, 6, 6, LETHAL_OBSTACLE)));
  for (int attempt = 0; attempt < 5; ++attempt) {
    const auto result = plan(make_goal());
    ASSERT_NE(result, nullptr);
    ASSERT_EQ(result->outcome, NavigationState::OUTCOME_REACHED) << result->message;
  }

  ASSERT_TRUE(publish_patch(make_patch(2, 6, 6, LETHAL_OBSTACLE)));
  ASSERT_TRUE(wait_until("the newer patch blocks the goal", [this]() {
    const auto result = plan(make_goal());
    return result != nullptr && result->outcome == NavigationState::OUTCOME_NO_PATH;
  }));
}

TEST_F(GlobalPathPlannerFixture, AWaitingGoalIsDisplacedByANewerOne)
{
  start({rclcpp::Parameter("tf_lookup_timeout", BLOCKING_TF_TIMEOUT)});
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  // The worker blocks in a lookup that cannot succeed, so the next two goals queue behind it.
  Action::Goal blocking = make_goal();
  blocking.use_start = false;
  const ClientGoalHandle::SharedPtr holder = send(blocking);
  ASSERT_NE(holder, nullptr);

  const ClientGoalHandle::SharedPtr displaced = send(make_goal());
  ASSERT_NE(displaced, nullptr);
  const ClientGoalHandle::SharedPtr newest = send(make_goal());
  ASSERT_NE(newest, nullptr);

  rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
  const auto result = wait_for_result(displaced, &code);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_CANCELED);
  EXPECT_NE(result->message.find("preempted"), std::string::npos) << result->message;
  EXPECT_TRUE(is_one_line(result->message));

  const auto newest_result = wait_for_result(newest);
  ASSERT_NE(newest_result, nullptr);
  EXPECT_EQ(newest_result->outcome, NavigationState::OUTCOME_REACHED) << newest_result->message;
}

TEST_F(GlobalPathPlannerFixture, ARunningGoalIsAbortedWhenANewerOneArrives)
{
  start({rclcpp::Parameter("tf_lookup_timeout", DEADLINE.count() * 1.0)});
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));

  Action::Goal waiting_for_tf = make_goal();
  waiting_for_tf.use_start = false;
  const ClientGoalHandle::SharedPtr running = send(waiting_for_tf);
  ASSERT_NE(running, nullptr);

  const ClientGoalHandle::SharedPtr newer = send(make_goal());
  ASSERT_NE(newer, nullptr);
  // The lookup only completes now, so the running goal reaches its next interruption point.
  broadcast("map", "base_footprint", cell_center(1), cell_center(1));

  rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
  const auto result = wait_for_result(running, &code);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_CANCELED);
  EXPECT_NE(result->message.find("preempted"), std::string::npos) << result->message;

  const auto newer_result = wait_for_result(newer);
  ASSERT_NE(newer_result, nullptr);
  EXPECT_EQ(newer_result->outcome, NavigationState::OUTCOME_REACHED) << newer_result->message;
}

TEST_F(GlobalPathPlannerFixture, ACanceledGoalIsCanceledAndPublishesNoPath)
{
  start({rclcpp::Parameter("tf_lookup_timeout", BLOCKING_TF_TIMEOUT)});
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  last_path_ = nullptr;

  Action::Goal blocking = make_goal();
  blocking.use_start = false;
  const ClientGoalHandle::SharedPtr holder = send(blocking);
  ASSERT_NE(holder, nullptr);

  const ClientGoalHandle::SharedPtr canceled = send(make_goal());
  ASSERT_NE(canceled, nullptr);
  auto cancel_future = client_->async_cancel_goal(canceled);
  ASSERT_EQ(cancel_future.wait_for(DEADLINE), std::future_status::ready);

  rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
  const auto result = wait_for_result(canceled, &code);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result->outcome, NavigationState::OUTCOME_CANCELED);
  EXPECT_TRUE(is_one_line(result->message));
  EXPECT_TRUE(result->path.poses.empty());
  EXPECT_EQ(last_path_, nullptr);
}

TEST_F(GlobalPathPlannerFixture, RefusesToStartOnAPlannerTypeNobodyDefined)
{
  try {
    start({rclcpp::Parameter("planner_type", "dubins")});
    FAIL() << "the node started with an unknown planner_type";
  } catch (const std::runtime_error & error) {
    const std::string line = error.what();
    EXPECT_NE(line.find("planner_type"), std::string::npos) << line;
    EXPECT_NE(line.find("hybrid_astar"), std::string::npos) << line;
    EXPECT_TRUE(is_one_line(line));
  }
}

TEST_F(GlobalPathPlannerFixture, RefusesToStartOnHybridValuesEltaninWouldThrowOn)
{
  EXPECT_THROW(start({rclcpp::Parameter("hybrid.heading_bins", 4)}), std::runtime_error);
}

TEST_F(GlobalPathPlannerFixture, HybridAStarPlansAndIsNamedInTheResult)
{
  start(
    {rclcpp::Parameter("planner_type", "hybrid_astar"),
     rclcpp::Parameter("hybrid.minimum_turning_radius", 0.2)});
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  last_path_ = nullptr;

  const auto result = plan(make_goal());
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->outcome, NavigationState::OUTCOME_REACHED) << result->message;
  EXPECT_NE(result->message.find("hybrid_astar"), std::string::npos) << result->message;
  ASSERT_TRUE(wait_until("the path is published", [this]() { return last_path_ != nullptr; }));
  EXPECT_FALSE(last_path_->poses.empty());
}

TEST_F(GlobalPathPlannerFixture, NoFootprintPublisherExistsUnlessItIsAskedFor)
{
  start({rclcpp::Parameter("publish_footprint_path", false)});
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  ASSERT_EQ(plan(make_goal())->outcome, NavigationState::OUTCOME_REACHED);
  EXPECT_EQ(helper_->count_publishers("/global_path_planner/footprint_path"), 0u);
}

TEST_F(GlobalPathPlannerFixture, TheFootprintIsLaidAlongThePathWhenItIsAskedFor)
{
  start(
    {rclcpp::Parameter("publish_footprint_path", true),
     rclcpp::Parameter("footprint_marker_stride", 2)});
  visualization_msgs::msg::MarkerArray::ConstSharedPtr markers;
  const auto subscription = helper_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/global_path_planner/footprint_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&markers](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) { markers = msg; });

  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  ASSERT_EQ(plan(make_goal())->outcome, NavigationState::OUTCOME_REACHED);
  ASSERT_TRUE(wait_until("the footprints arrive", [&markers]() { return markers != nullptr; }));

  ASSERT_GE(markers->markers.size(), 2u);
  EXPECT_EQ(markers->markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  const auto & outline = markers->markers[1];
  EXPECT_EQ(outline.type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(outline.header.frame_id, "map");
  // The default footprint has four vertices and the strip repeats the first to close it.
  EXPECT_EQ(outline.points.size(), 5u);
}

/// AC-20 is not directly observable, so the stand-in is a costmap that lands between two plans.
TEST_F(GlobalPathPlannerFixture, TheCostmapSubscriptionSurvivesAPlan)
{
  start();
  ASSERT_TRUE(publish_costmap(make_costmap_msg(1)));
  ASSERT_EQ(plan(make_goal())->outcome, NavigationState::OUTCOME_REACHED);

  eltanin_msgs::msg::Costmap blocked = make_costmap_msg(2);
  block_cell(blocked, 6, 6);
  costmap_publisher_->publish(blocked);
  ASSERT_TRUE(wait_until("the second costmap reaches the planner", [this]() {
    const auto result = plan(make_goal());
    return result != nullptr && result->outcome == NavigationState::OUTCOME_NO_PATH;
  }));
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
