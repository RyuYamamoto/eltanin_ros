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

#include "eltanin_controller/path_follower.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eltanin_ros_common/geometry_conversion.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eltanin_msgs/msg/directed_path.hpp>
#include <eltanin_msgs/msg/follower_diagnostic.hpp>
#include <eltanin_msgs/msg/trajectory2_d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using eltanin_controller::PathFollower;
using Diagnostic = eltanin_msgs::msg::FollowerDiagnostic;

/// Long enough that a loaded CI machine still gets there, short enough to fail rather than hang.
constexpr std::chrono::seconds DEADLINE{10};
constexpr std::chrono::milliseconds POLL_INTERVAL{2};

rclcpp::QoS control_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
}

/// Frames of this test alone: /tf_static is latched, so "map" may already exist in the world.
constexpr const char * MAP_FRAME = "test_map";
constexpr const char * BASE_FRAME = "test_base";

/// Names of this test alone, so a planner test running beside it cannot feed the follower.
constexpr const char * PATH_TOPIC = "/test_follower_path";
constexpr const char * TRAJECTORY_TOPIC = "/test_follower_trajectory";
constexpr const char * DIRECTED_PATH_TOPIC = "/test_follower_directed_path";

nav_msgs::msg::Path make_path(
  const rclcpp::Time & stamp, const std::string & frame_id = MAP_FRAME, std::size_t poses = 41,
  double goal_yaw = 0.0)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.poses.resize(poses);
  for (std::size_t i = 0; i < poses; ++i) {
    msg.poses[i].header = msg.header;
    msg.poses[i].pose.position.x = 0.05 * static_cast<double>(i);
    msg.poses[i].pose.orientation =
      eltanin_ros_common::to_quaternion(i + 1 == poses ? goal_yaw : 0.0);
  }
  return msg;
}

eltanin_msgs::msg::Trajectory2D make_trajectory(const rclcpp::Time & stamp)
{
  eltanin_msgs::msg::Trajectory2D msg;
  msg.header.frame_id = MAP_FRAME;
  msg.header.stamp = stamp;
  for (std::size_t i = 0; i < 41; ++i) {
    eltanin_msgs::msg::TrajectoryPoint2D point;
    point.x = 0.05 * static_cast<double>(i);
    msg.points.push_back(point);
  }
  return msg;
}

/// Straight out and then straight back along the same line: one cusp, at pose `forward`.
eltanin_msgs::msg::DirectedPath make_directed_path(
  const rclcpp::Time & stamp, bool reversing, std::size_t forward = 20, std::size_t back = 10)
{
  eltanin_msgs::msg::DirectedPath msg;
  msg.header.frame_id = MAP_FRAME;
  msg.header.stamp = stamp;
  const std::size_t total = reversing ? forward + back : forward;
  for (std::size_t i = 0; i <= total; ++i) {
    const double travelled =
      i <= forward ? 0.05 * static_cast<double>(i) : 0.05 * static_cast<double>(2 * forward - i);
    geometry_msgs::msg::Pose pose;
    pose.position.x = travelled;
    pose.orientation = eltanin_ros_common::to_quaternion(0.0);
    msg.poses.push_back(pose);
    if (i > 0) {
      msg.segment_directions.push_back(
        i <= forward ? eltanin_msgs::msg::DirectedPath::DIRECTION_FORWARD
                     : eltanin_msgs::msg::DirectedPath::DIRECTION_REVERSE);
    }
  }
  if (!reversing) {
    msg.segment_directions.clear();
  }
  return msg;
}

std::vector<rclcpp::Parameter> shipped_configuration()
{
  std::vector<rclcpp::Parameter> parameters;
  const std::vector<std::string> files{
    ament_index_cpp::get_package_share_directory("eltanin_ros_common") +
      "/config/robot/kachaka.yaml",
    ament_index_cpp::get_package_share_directory("eltanin_controller") +
      "/config/path_follower.param.yaml"};
  for (const std::string & file : files) {
    for (const auto & [node_name, values] : rclcpp::parameter_map_from_yaml_file(file)) {
      (void)node_name;
      parameters.insert(parameters.end(), values.begin(), values.end());
    }
  }
  return parameters;
}

/// One MultiThreadedExecutor for both nodes, which is the executor the generated entry point uses.
class PathFollowerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    path_publisher_ = helper_->create_publisher<nav_msgs::msg::Path>(PATH_TOPIC, control_qos());
    trajectory_publisher_ =
      helper_->create_publisher<eltanin_msgs::msg::Trajectory2D>(TRAJECTORY_TOPIC, control_qos());
    directed_path_publisher_ = helper_->create_publisher<eltanin_msgs::msg::DirectedPath>(
      DIRECTED_PATH_TOPIC, control_qos());
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    command_subscription_ = helper_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/test_path_follower/cmd_vel_raw", control_qos(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        commands_.push_back(*msg);
      });
    diagnostic_subscription_ = helper_->create_subscription<Diagnostic>(
      "/test_path_follower/follower_state", control_qos(), [this](Diagnostic::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.push_back(*msg);
      });
    lookahead_subscription_ = helper_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/test_path_follower/lookahead_point", control_qos(),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        lookaheads_.push_back(*msg);
      });
    reset_client_ = helper_->create_client<std_srvs::srv::Trigger>("/test_path_follower/reset");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(helper_);
    spinner_ = std::thread([this]() { executor_->spin(); });
    while (!executor_->is_spinning()) {
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
  }

  void TearDown() override
  {
    executor_->cancel();
    spinner_.join();
    reset_client_.reset();
    node_.reset();
    broadcaster_.reset();
    helper_.reset();
    executor_.reset();
  }

  void start(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    // The shipped configuration, so a key missing from it fails here instead of on the robot.
    std::vector<rclcpp::Parameter> all = shipped_configuration();
    all.emplace_back("frames.map", MAP_FRAME);
    all.emplace_back("frames.base", BASE_FRAME);
    all.insert(all.end(), parameters.begin(), parameters.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(all);
    options.arguments(
      {"--ros-args", "-r", "__node:=test_path_follower", "-r",
       std::string("global_path_planner/global_path:=") + PATH_TOPIC, "-r",
       std::string("local_path_planner/local_trajectory:=") + TRAJECTORY_TOPIC, "-r",
       std::string("global_path_planner/global_path_directed:=") + DIRECTED_PATH_TOPIC});
    node_ = std::make_shared<PathFollower>(options);
    executor_->add_node(node_);
  }

  void broadcast_robot(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = MAP_FRAME;
    transform.header.stamp = helper_->now();
    transform.child_frame_id = BASE_FRAME;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation = eltanin_ros_common::to_quaternion(yaw);
    broadcaster_->sendTransform(transform);
  }

  ::testing::AssertionResult wait_until(
    const std::string & what, const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + DEADLINE;
    while (std::chrono::steady_clock::now() < deadline) {
      if (done()) {
        return ::testing::AssertionSuccess();
      }
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
    return ::testing::AssertionFailure() << what << " did not happen within the deadline";
  }

  /// True once `count` more commands have been published than there were at the call.
  ::testing::AssertionResult wait_for_commands(std::size_t count)
  {
    const std::size_t target = command_count() + count;
    return wait_until("another " + std::to_string(count) + " commands", [this, target]() {
      return command_count() >= target;
    });
  }

  /// A volatile publisher drops what it sends before the subscription is matched.
  ::testing::AssertionResult publish_path(const nav_msgs::msg::Path & msg)
  {
    const auto linked = wait_until("the path subscription to appear", [this]() {
      return path_publisher_->get_subscription_count() > 0;
    });
    if (!linked) {
      return linked;
    }
    path_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult publish_directed_path(const eltanin_msgs::msg::DirectedPath & msg)
  {
    const auto linked = wait_until("the directed path subscription to appear", [this]() {
      return directed_path_publisher_->get_subscription_count() > 0;
    });
    if (!linked) {
      return linked;
    }
    directed_path_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult publish_trajectory(const eltanin_msgs::msg::Trajectory2D & msg)
  {
    const auto linked = wait_until("the trajectory subscription to appear", [this]() {
      return trajectory_publisher_->get_subscription_count() > 0;
    });
    if (!linked) {
      return linked;
    }
    trajectory_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult wait_for_reason(std::uint8_t reason)
  {
    const auto reached =
      wait_until("the reason becomes " + std::to_string(reason), [this, reason]() {
        const Diagnostic latest = last_diagnostic();
        return latest.header.stamp.sec != 0 && latest.reason == reason;
      });
    if (reached) {
      return reached;
    }
    const Diagnostic latest = last_diagnostic();
    return ::testing::AssertionFailure()
           << "the reason stayed " << static_cast<int>(latest.reason) << " ('" << latest.message
           << "') instead of becoming " << static_cast<int>(reason);
  }

  std::size_t command_count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return commands_.size();
  }

  geometry_msgs::msg::TwistStamped last_command() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return commands_.empty() ? geometry_msgs::msg::TwistStamped{} : commands_.back();
  }

  Diagnostic last_diagnostic() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return diagnostics_.empty() ? Diagnostic{} : diagnostics_.back();
  }

  std::size_t lookahead_count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return lookaheads_.size();
  }

  /// The largest linear.x seen so far, which is where the velocity ramp had got to.
  double peak_linear() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    double peak = 0.0;
    for (const geometry_msgs::msg::TwistStamped & command : commands_) {
      peak = std::max(peak, command.twist.linear.x);
    }
    return peak;
  }

  bool every_command_is_zero() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const geometry_msgs::msg::TwistStamped & command : commands_) {
      if (command.twist.linear.x != 0.0 || command.twist.angular.z != 0.0) {
        return false;
      }
    }
    return true;
  }

  void clear()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
    diagnostics_.clear();
    lookaheads_.clear();
  }

  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<PathFollower> node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<eltanin_msgs::msg::Trajectory2D>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<eltanin_msgs::msg::DirectedPath>::SharedPtr directed_path_publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<Diagnostic>::SharedPtr diagnostic_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

  mutable std::mutex mutex_;
  std::vector<geometry_msgs::msg::TwistStamped> commands_;
  std::vector<Diagnostic> diagnostics_;
  std::vector<geometry_msgs::msg::PointStamped> lookaheads_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
};

TEST_F(PathFollowerFixture, WithoutAPathTheZeroCommandKeepsComing)
{
  start();
  ASSERT_TRUE(wait_for_commands(5));
  ASSERT_TRUE(wait_for_reason(Diagnostic::REASON_NO_INPUT));

  EXPECT_TRUE(every_command_is_zero());
  const Diagnostic latest = last_diagnostic();
  EXPECT_TRUE(latest.ok);
  EXPECT_EQ(latest.status, Diagnostic::STATUS_NO_PATH);
  EXPECT_FALSE(latest.message.empty());
  EXPECT_EQ(latest.message.find('\n'), std::string::npos);
}

TEST_F(PathFollowerFixture, TheFirstCycleHasNoElapsedTimeAndSaysSo)
{
  start();
  ASSERT_TRUE(wait_for_commands(1));
  ASSERT_TRUE(
    wait_until("the first diagnostic", [this]() { return last_diagnostic().reason != 0; }));

  const std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_FALSE(diagnostics_.empty());
  EXPECT_EQ(diagnostics_.front().reason, Diagnostic::REASON_NO_DT);
  EXPECT_EQ(diagnostics_.front().control_dt, 0.0);
  EXPECT_TRUE(diagnostics_.front().ok);
}

TEST_F(PathFollowerFixture, WithoutATransformTheZeroCommandKeepsComing)
{
  start();
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now())));
  ASSERT_TRUE(wait_for_reason(Diagnostic::REASON_NO_TRANSFORM));

  clear();
  ASSERT_TRUE(wait_for_commands(5));
  EXPECT_TRUE(every_command_is_zero());
  EXPECT_TRUE(last_diagnostic().ok);
}

TEST_F(PathFollowerFixture, APathAndATransformProduceAForwardCommandAndALookaheadPoint)
{
  start();
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now())));

  ASSERT_TRUE(wait_until("a forward command", [this]() { return peak_linear() > 0.0; }));
  ASSERT_TRUE(wait_until("a lookahead point", [this]() { return lookahead_count() > 0; }));

  const Diagnostic latest = last_diagnostic();
  EXPECT_EQ(latest.reason, Diagnostic::REASON_NONE);
  EXPECT_EQ(latest.status, Diagnostic::STATUS_TRACKING);
  EXPECT_TRUE(latest.ok);
  EXPECT_GT(latest.control_dt, 0.0);
  EXPECT_EQ(last_command().header.frame_id, BASE_FRAME);
}

TEST_F(PathFollowerFixture, APathWithoutADeadlineNeverGoesStale)
{
  start({rclcpp::Parameter("path_timeout", 0.0)});
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now())));
  ASSERT_TRUE(wait_until("tracking", [this]() { return peak_linear() > 0.0; }));

  clear();
  ASSERT_TRUE(wait_for_commands(40));
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const Diagnostic & diagnostic : diagnostics_) {
    EXPECT_NE(diagnostic.reason, Diagnostic::REASON_INPUT_STALE);
  }
}

TEST_F(PathFollowerFixture, ATrajectoryThatStopsArrivingGoesStaleAndTheCommandKeepsComing)
{
  start(
    {rclcpp::Parameter("path_source", "trajectory"), rclcpp::Parameter("trajectory_timeout", 0.2)});
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_trajectory(make_trajectory(helper_->now())));
  ASSERT_TRUE(wait_until("tracking", [this]() { return peak_linear() > 0.0; }));

  ASSERT_TRUE(wait_for_reason(Diagnostic::REASON_INPUT_STALE));
  clear();
  ASSERT_TRUE(wait_for_commands(5));
  EXPECT_TRUE(every_command_is_zero());
  EXPECT_TRUE(last_diagnostic().ok);
}

TEST_F(PathFollowerFixture, APathInAnotherFrameIsRejectedAndNotFollowed)
{
  start();
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now(), "odom")));

  ASSERT_TRUE(wait_for_reason(Diagnostic::REASON_INPUT_REJECTED));
  EXPECT_TRUE(every_command_is_zero());
  EXPECT_TRUE(last_diagnostic().ok);
  EXPECT_NE(last_diagnostic().message.find("odom"), std::string::npos);
}

TEST_F(PathFollowerFixture, AnEmptyPathIsItsOwnReason)
{
  start();
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now(), MAP_FRAME, 0)));

  ASSERT_TRUE(wait_for_reason(Diagnostic::REASON_INPUT_EMPTY));
  EXPECT_TRUE(every_command_is_zero());
  EXPECT_TRUE(last_diagnostic().ok);
}

TEST_F(PathFollowerFixture, ResetTakesEffectBeforeTheNextCommandAndTheRampStartsAgain)
{
  start();
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now())));
  ASSERT_TRUE(wait_until("the ramp to climb", [this]() { return peak_linear() > 0.1; }));

  ASSERT_TRUE(reset_client_->wait_for_service(DEADLINE));
  auto future =
    reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(future.wait_for(DEADLINE), std::future_status::ready);
  const auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->message.empty());

  // The reset drops the path along with the latches, so the follower falls back to no input.
  ASSERT_TRUE(wait_for_reason(Diagnostic::REASON_NO_INPUT));
  EXPECT_DOUBLE_EQ(last_command().twist.linear.x, 0.0);
}

TEST_F(PathFollowerFixture, AnAlignmentThatNeverFinishesIsReportedAsNotOk)
{
  start({rclcpp::Parameter("yaw_align_timeout", 0.2)});
  // At the goal position with the goal yaw a quarter turn away: nothing but the timer moves.
  broadcast_robot(2.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_path(make_path(helper_->now(), MAP_FRAME, 41, 1.57)));

  ASSERT_TRUE(wait_until("the alignment to give up", [this]() {
    return last_diagnostic().approach_state == Diagnostic::APPROACH_ALIGNMENT_TIMEOUT;
  }));
  const Diagnostic latest = last_diagnostic();
  EXPECT_FALSE(latest.ok);
  EXPECT_EQ(latest.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_DOUBLE_EQ(last_command().twist.angular.z, 0.0);
}

TEST_F(PathFollowerFixture, ADirectedPathIsFollowedLikeAnyOtherWhenItOnlyGoesForward)
{
  start({rclcpp::Parameter("path_source", "directed_path")});
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_directed_path(make_directed_path(helper_->now(), false)));

  ASSERT_TRUE(wait_until("a forward command", [this]() { return peak_linear() > 0.0; }));
  const Diagnostic latest = last_diagnostic();
  EXPECT_EQ(latest.status, Diagnostic::STATUS_TRACKING);
  EXPECT_EQ(latest.travel_direction, Diagnostic::DIRECTION_FORWARD);
  EXPECT_TRUE(latest.ok);
}

TEST_F(PathFollowerFixture, PurePursuitRefusesAReversingPathAndKeepsPublishingZero)
{
  start({rclcpp::Parameter("path_source", "directed_path")});
  broadcast_robot(0.0, 0.0, 0.0);
  ASSERT_TRUE(wait_for_commands(2));
  ASSERT_TRUE(publish_directed_path(make_directed_path(helper_->now(), true)));

  ASSERT_TRUE(wait_until("the follower to refuse the path", [this]() {
    return last_diagnostic().status == Diagnostic::STATUS_PATH_NOT_SUPPORTED;
  }));
  clear();
  ASSERT_TRUE(wait_for_commands(5));
  EXPECT_TRUE(every_command_is_zero());
  const Diagnostic latest = last_diagnostic();
  EXPECT_EQ(latest.reason, Diagnostic::REASON_CONTROLLER);
  EXPECT_FALSE(latest.ok);
}

TEST_F(PathFollowerFixture, AParameterOutsideItsRangeStopsTheNodeFromStarting)
{
  EXPECT_THROW(
    start({rclcpp::Parameter("pure_pursuit.min_lookahead_dist", 0.0)}), std::runtime_error);
  EXPECT_THROW(start({rclcpp::Parameter("path_source", "bogus")}), std::runtime_error);
  EXPECT_THROW(start({rclcpp::Parameter("update_frequency", 0.0)}), std::runtime_error);
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
