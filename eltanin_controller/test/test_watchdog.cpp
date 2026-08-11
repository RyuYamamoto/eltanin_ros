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

#include "eltanin_controller/collision_predictor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin_ros_common/geometry_conversion.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <eltanin_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace
{

using eltanin_controller::CollisionPredictor;
using Diagnostic = diagnostic_msgs::msg::DiagnosticArray;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

constexpr std::chrono::seconds DEADLINE{10};
constexpr std::chrono::milliseconds POLL_INTERVAL{2};

/// Frames of this test alone: /tf_static is latched, so "map" may already exist in the world.
constexpr const char * MAP_FRAME = "test_limiter_map";
constexpr const char * BASE_FRAME = "test_limiter_base";

/// Names of this test alone, so a follower test running beside it cannot feed the limiter.
constexpr const char * COMMAND_TOPIC = "/test_limiter_cmd_raw";
constexpr const char * MAP_TOPIC = "/test_limiter_local_map";
constexpr const char * CMD_VEL_TOPIC = "/test_limiter_cmd_vel";
constexpr const char * NODE_NAME = "test_collision_predictor";

/// 4 m square at 0.05 m, so a 0.65 m rollout from the middle stays well inside it.
constexpr int MAP_CELLS = 80;
constexpr double RESOLUTION = 0.05;

rclcpp::QoS control_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
}

Status status_of(const Diagnostic & msg)
{
  return msg.status.empty() ? Status{} : msg.status.front();
}

std::string value_of(const Diagnostic & msg, const std::string & key)
{
  for (const diagnostic_msgs::msg::KeyValue & pair : status_of(msg).values) {
    if (pair.key == key) {
      return pair.value;
    }
  }
  return "";
}

double number_of(const Diagnostic & msg, const std::string & key)
{
  const std::string value = value_of(msg, key);
  if (value == "inf") {
    return std::numeric_limits<double>::infinity();
  }
  if (value.empty() || value == "nan") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::stod(value);
}

geometry_msgs::msg::TwistStamped make_command(
  const rclcpp::Time & stamp, double linear, double angular,
  const std::string & frame_id = BASE_FRAME)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.twist.linear.x = linear;
  msg.twist.angular.z = angular;
  return msg;
}

/// A free local map with one optional lethal column, the shape local_map will publish (D-14).
eltanin_msgs::msg::Costmap make_map(
  const rclcpp::Time & stamp, int wall_x = -1, const std::string & frame_id = MAP_FRAME)
{
  eltanin_msgs::msg::Costmap msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.info.resolution = RESOLUTION;
  msg.info.width = static_cast<std::uint32_t>(MAP_CELLS);
  msg.info.height = static_cast<std::uint32_t>(MAP_CELLS);
  msg.info.origin_x = 0.0;
  msg.info.origin_y = 0.0;
  msg.data.assign(
    static_cast<std::size_t>(MAP_CELLS) * static_cast<std::size_t>(MAP_CELLS),
    eltanin::map::FREE_SPACE);
  if (wall_x >= 0) {
    for (int my = 0; my < MAP_CELLS; ++my) {
      msg.data[static_cast<std::size_t>(my * MAP_CELLS + wall_x)] = eltanin::map::LETHAL_OBSTACLE;
    }
  }
  return msg;
}

std::vector<rclcpp::Parameter> parameters_from(const std::string & file)
{
  std::vector<rclcpp::Parameter> parameters;
  for (const auto & [node_name, values] : rclcpp::parameter_map_from_yaml_file(file)) {
    (void)node_name;
    parameters.insert(parameters.end(), values.begin(), values.end());
  }
  return parameters;
}

std::string profile_file()
{
  return ament_index_cpp::get_package_share_directory("eltanin_ros_common") +
         "/config/robot/kachaka.yaml";
}

std::string config_file()
{
  return ament_index_cpp::get_package_share_directory("eltanin_controller") +
         "/config/collision_predictor.param.yaml";
}

class CollisionPredictorFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<rclcpp::Node>("test_limiter_helper");
    command_publisher_ =
      helper_->create_publisher<geometry_msgs::msg::TwistStamped>(COMMAND_TOPIC, control_qos());
    map_publisher_ =
      helper_->create_publisher<eltanin_msgs::msg::Costmap>(MAP_TOPIC, control_qos());
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    cmd_vel_subscription_ = helper_->create_subscription<geometry_msgs::msg::Twist>(
      CMD_VEL_TOPIC, control_qos(), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        commands_.push_back(*msg);
      });
    diagnostic_subscription_ = helper_->create_subscription<Diagnostic>(
      std::string("/") + NODE_NAME + "/diagnostics", control_qos(),
      [this](Diagnostic::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.push_back(*msg);
      });
    path_subscription_ = helper_->create_subscription<nav_msgs::msg::Path>(
      std::string("/") + NODE_NAME + "/predicted_poses", control_qos(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        paths_.push_back(*msg);
      });
    footprint_subscription_ = helper_->create_subscription<geometry_msgs::msg::PolygonStamped>(
      std::string("/") + NODE_NAME + "/footprint", control_qos(),
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        footprints_.push_back(*msg);
      });
    enable_client_ = helper_->create_client<std_srvs::srv::SetBool>(
      std::string("/") + NODE_NAME + "/enable_output");

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
    enable_client_.reset();
    node_.reset();
    broadcaster_.reset();
    helper_.reset();
    executor_.reset();
  }

  void start(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    std::vector<rclcpp::Parameter> all = parameters_from(profile_file());
    const std::vector<rclcpp::Parameter> shipped = parameters_from(config_file());
    all.insert(all.end(), shipped.begin(), shipped.end());
    all.emplace_back("frames.map", MAP_FRAME);
    all.emplace_back("frames.base", BASE_FRAME);
    all.insert(all.end(), parameters.begin(), parameters.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(all);
    options.arguments(
      {"--ros-args", "-r", std::string("__node:=") + NODE_NAME, "-r",
       std::string("path_follower/cmd_vel_raw:=") + COMMAND_TOPIC, "-r",
       std::string("local_map/local_map:=") + MAP_TOPIC, "-r",
       std::string("cmd_vel:=") + CMD_VEL_TOPIC});
    node_ = std::make_shared<CollisionPredictor>(options);
    executor_->add_node(node_);
  }

  /// Nothing reaches /cmd_vel before this; the startup default is the output being off.
  ::testing::AssertionResult enable_output(bool enabled)
  {
    if (!enable_client_->wait_for_service(std::chrono::seconds(5))) {
      return ::testing::AssertionFailure() << "~/enable_output never appeared";
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = enable_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return ::testing::AssertionFailure() << "~/enable_output did not answer";
    }
    return future.get()->success ? ::testing::AssertionSuccess()
                                 : ::testing::AssertionFailure() << "~/enable_output refused";
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

  ::testing::AssertionResult wait_for_reason(const std::string & reason)
  {
    const auto reached = wait_until("the reason becomes " + reason, [this, &reason]() {
      return value_of(last_diagnostic(), "reason") == reason;
    });
    if (reached) {
      return reached;
    }
    return ::testing::AssertionFailure()
           << "the reason stayed '" << value_of(last_diagnostic(), "reason") << "' ('"
           << status_of(last_diagnostic()).message << "') instead of becoming '" << reason << "'";
  }

  ::testing::AssertionResult wait_for_commands(std::size_t count)
  {
    const std::size_t target = command_count() + count;
    return wait_until("another " + std::to_string(count) + " commands", [this, target]() {
      return command_count() >= target;
    });
  }

  ::testing::AssertionResult wait_for_diagnostics(std::size_t count)
  {
    const std::size_t target = diagnostic_count() + count;
    return wait_until("another " + std::to_string(count) + " diagnostics", [this, target]() {
      return diagnostic_count() >= target;
    });
  }

  /// A volatile publisher drops what it sends before the subscription is matched.
  ::testing::AssertionResult publish_command(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto linked = wait_until("the command subscription to appear", [this]() {
      return command_publisher_->get_subscription_count() > 0;
    });
    if (!linked) {
      return linked;
    }
    command_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult publish_map(const eltanin_msgs::msg::Costmap & msg)
  {
    const auto linked = wait_until("the map subscription to appear", [this]() {
      return map_publisher_->get_subscription_count() > 0;
    });
    if (!linked) {
      return linked;
    }
    map_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  /// Keeps both inputs fresh over several cycles; the deadlines are shorter than any wait here.
  ::testing::AssertionResult drive(double linear, double angular, int wall_x, int cycles)
  {
    for (int cycle = 0; cycle < cycles; ++cycle) {
      const rclcpp::Time now = helper_->now();
      const auto command = publish_command(make_command(now, linear, angular));
      if (!command) {
        return command;
      }
      const auto map = publish_map(make_map(now, wall_x));
      if (!map) {
        return map;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return ::testing::AssertionSuccess();
  }

  std::size_t command_count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return commands_.size();
  }

  std::size_t diagnostic_count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return diagnostics_.size();
  }

  geometry_msgs::msg::Twist last_command() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return commands_.empty() ? geometry_msgs::msg::Twist{} : commands_.back();
  }

  Diagnostic last_diagnostic() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return diagnostics_.empty() ? Diagnostic{} : diagnostics_.back();
  }

  nav_msgs::msg::Path last_path() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return paths_.empty() ? nav_msgs::msg::Path{} : paths_.back();
  }

  geometry_msgs::msg::PolygonStamped last_footprint() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return footprints_.empty() ? geometry_msgs::msg::PolygonStamped{} : footprints_.back();
  }

  bool every_command_is_zero() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return std::all_of(
      commands_.begin(), commands_.end(), [](const geometry_msgs::msg::Twist & command) {
        return command.linear.x == 0.0 && command.angular.z == 0.0;
      });
  }

  double peak_linear() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    double peak = 0.0;
    for (const geometry_msgs::msg::Twist & command : commands_) {
      peak = std::max(peak, std::abs(command.linear.x));
    }
    return peak;
  }

  void clear()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
    diagnostics_.clear();
    paths_.clear();
    footprints_.clear();
  }

  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<CollisionPredictor> node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<eltanin_msgs::msg::Costmap>::SharedPtr map_publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<Diagnostic>::SharedPtr diagnostic_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;

  mutable std::mutex mutex_;
  std::vector<geometry_msgs::msg::Twist> commands_;
  std::vector<Diagnostic> diagnostics_;
  std::vector<nav_msgs::msg::Path> paths_;
  std::vector<geometry_msgs::msg::PolygonStamped> footprints_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
};

}  // namespace

TEST_F(CollisionPredictorFixture, StartsOnTheRobotProfileAloneWithNoNodeConfiguration)
{
  std::vector<rclcpp::Parameter> profile = parameters_from(profile_file());
  profile.emplace_back("frames.map", MAP_FRAME);
  profile.emplace_back("frames.base", BASE_FRAME);

  rclcpp::NodeOptions options;
  options.parameter_overrides(profile);
  options.arguments({"--ros-args", "-r", std::string("__node:=") + NODE_NAME});

  ASSERT_NO_THROW(node_ = std::make_shared<CollisionPredictor>(options));
  EXPECT_FALSE(node_->get_parameter("output_enabled_on_startup").as_bool());
  EXPECT_DOUBLE_EQ(node_->get_parameter("update_frequency").as_double(), 20.0);
}

TEST_F(CollisionPredictorFixture, TheShippedConfigurationMatchesTheDeclaredKeys)
{
  start();

  std::set<std::string> shipped;
  for (const rclcpp::Parameter & value : parameters_from(config_file())) {
    shipped.insert(value.get_name());
  }
  const std::vector<std::string> declared = eltanin_controller::limiter::declared_keys();

  EXPECT_EQ(shipped.size(), declared.size());
  for (const std::string & key : declared) {
    EXPECT_EQ(shipped.count(key), 1u) << key << " is declared but missing from the shipped yaml";
    EXPECT_TRUE(node_->has_parameter(key)) << key;
  }
}

TEST_F(CollisionPredictorFixture, DisabledOutputPublishesNothingButKeepsTalking)
{
  start();
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 5));
  ASSERT_TRUE(wait_for_diagnostics(5));

  EXPECT_EQ(command_count(), 0u);
  EXPECT_EQ(value_of(last_diagnostic(), "reason"), "output_disabled");
  EXPECT_EQ(value_of(last_diagnostic(), "output_enabled"), "false");
  EXPECT_EQ(value_of(last_diagnostic(), "published"), "false");
  EXPECT_EQ(status_of(last_diagnostic()).level, Status::WARN);
}

TEST_F(CollisionPredictorFixture, ADisabledCycleIsStillACheckedCycle)
{
  // The point of a pre-flight check: everything but the publish has to be readable before the
  // robot can move, or the only way to find out that tf is missing is to enable the output.
  start();
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));
  ASSERT_TRUE(wait_until("a checked disabled cycle", [this]() {
    return value_of(last_diagnostic(), "transform_ok") == "true";
  }));

  const Diagnostic latest = last_diagnostic();
  EXPECT_EQ(command_count(), 0u);
  EXPECT_EQ(value_of(latest, "reason"), "output_disabled");
  EXPECT_EQ(value_of(latest, "output_enabled"), "false");
  EXPECT_EQ(value_of(latest, "clearance_available"), "true");
  EXPECT_FALSE(std::isnan(number_of(latest, "clearance")));
  EXPECT_GT(number_of(latest, "horizon"), 0.0);
  EXPECT_DOUBLE_EQ(number_of(latest, "requested_linear"), 0.25);
  // The command it would have published is not reported as one it did.
  EXPECT_DOUBLE_EQ(number_of(latest, "command_linear"), 0.0);
  EXPECT_EQ(last_path().poses.size(), 11u);
}

TEST_F(CollisionPredictorFixture, ADisabledCycleReportsAMissingTransform)
{
  start();
  ASSERT_TRUE(drive(0.25, 0.0, -1, 5));
  ASSERT_TRUE(wait_for_diagnostics(3));

  EXPECT_EQ(value_of(last_diagnostic(), "reason"), "output_disabled");
  EXPECT_EQ(value_of(last_diagnostic(), "transform_ok"), "false");
  EXPECT_EQ(command_count(), 0u);
}

TEST_F(CollisionPredictorFixture, EnabledWithNoInputsPublishesZeroAtTheCycleRate)
{
  start();
  ASSERT_TRUE(enable_output(true));

  ASSERT_TRUE(wait_for_commands(5));

  EXPECT_TRUE(every_command_is_zero());
  EXPECT_EQ(value_of(last_diagnostic(), "reason"), "command_missing");
  EXPECT_EQ(status_of(last_diagnostic()).level, Status::STALE);
}

TEST_F(CollisionPredictorFixture, MissingMapZeroesTheCommand)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(publish_command(make_command(helper_->now(), 0.25, 0.0)));

  ASSERT_TRUE(wait_for_reason("map_missing"));
  clear();
  ASSERT_TRUE(wait_for_commands(3));
  EXPECT_TRUE(every_command_is_zero());
}

TEST_F(CollisionPredictorFixture, StaleCommandZeroesTheCommand)
{
  start({rclcpp::Parameter("cmd_timeout", 0.2)});
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 5));
  ASSERT_TRUE(wait_until("a non-zero command", [this]() { return peak_linear() > 0.0; }));

  ASSERT_TRUE(wait_for_reason("command_stale"));
  clear();
  ASSERT_TRUE(wait_for_commands(3));
  EXPECT_TRUE(every_command_is_zero());
}

TEST_F(CollisionPredictorFixture, AFutureStampIsStaleToo)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(publish_map(make_map(helper_->now())));
  ASSERT_TRUE(publish_command(
    make_command(helper_->now() + rclcpp::Duration::from_seconds(10.0), 0.25, 0.0)));

  ASSERT_TRUE(wait_for_reason("command_stale"));
  EXPECT_LT(number_of(last_diagnostic(), "command_age"), 0.0);
}

TEST_F(CollisionPredictorFixture, WithoutATransformTheCommandIsZeroUntilTfArrives)
{
  // tf2 keeps the latest sample for a time-zero lookup, so the loss itself is not reproducible.
  start();
  ASSERT_TRUE(enable_output(true));
  ASSERT_TRUE(drive(0.25, 0.0, -1, 5));

  ASSERT_TRUE(wait_for_reason("no_transform"));
  EXPECT_EQ(value_of(last_diagnostic(), "transform_ok"), "false");
  EXPECT_EQ(status_of(last_diagnostic()).level, Status::WARN);
  clear();
  ASSERT_TRUE(wait_for_commands(3));
  EXPECT_TRUE(every_command_is_zero());

  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 10));
  EXPECT_TRUE(wait_until("a non-zero command", [this]() { return peak_linear() > 0.0; }));
}

TEST_F(CollisionPredictorFixture, NoCollisionPassesTheRequestThrough)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.1, -1, 8));

  ASSERT_TRUE(wait_for_reason("none"));
  EXPECT_EQ(status_of(last_diagnostic()).level, Status::OK);
  EXPECT_DOUBLE_EQ(number_of(last_diagnostic(), "command_linear"), 0.25);
  EXPECT_DOUBLE_EQ(number_of(last_diagnostic(), "command_angular"), 0.1);
  EXPECT_TRUE(std::isinf(number_of(last_diagnostic(), "collision_distance")));
  EXPECT_TRUE(std::isinf(number_of(last_diagnostic(), "time_to_collision")));
  EXPECT_DOUBLE_EQ(number_of(last_diagnostic(), "proximity_scale"), 1.0);
  EXPECT_DOUBLE_EQ(last_command().linear.x, 0.25);
}

TEST_F(CollisionPredictorFixture, TheExactCheckIsOnAndTheClearanceIsAvailable)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));
  ASSERT_TRUE(wait_for_reason("none"));

  EXPECT_EQ(value_of(last_diagnostic(), "exact_footprint_check"), "true");
  EXPECT_EQ(value_of(last_diagnostic(), "clearance_available"), "true");
  EXPECT_FALSE(std::isnan(number_of(last_diagnostic(), "clearance")));
}

TEST_F(CollisionPredictorFixture, TheHorizonFollowsTheRequestedSpeed)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);

  ASSERT_TRUE(drive(0.1, 0.0, -1, 8));
  ASSERT_TRUE(wait_until("the slow horizon", [this]() {
    return std::abs(number_of(last_diagnostic(), "horizon") - (0.3 + 0.1 / 0.5)) < 1e-4;
  }));

  ASSERT_TRUE(drive(0.3, 0.0, -1, 8));
  EXPECT_TRUE(wait_until("the fast horizon", [this]() {
    return std::abs(number_of(last_diagnostic(), "horizon") - (0.3 + 0.3 / 0.5)) < 1e-4;
  }));
}

TEST_F(CollisionPredictorFixture, TheLimitDoesNotRatchetToZero)
{
  start({rclcpp::Parameter("cmd_timeout", 0.2)});
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));
  ASSERT_TRUE(wait_until("a non-zero command", [this]() { return peak_linear() > 0.0; }));
  const double before = last_command().linear.x;

  ASSERT_TRUE(wait_for_reason("command_stale"));
  ASSERT_TRUE(wait_until("a zero command", [this]() { return last_command().linear.x == 0.0; }));

  // navyu wrote the limited value back into the input, so the request never came back.
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));
  ASSERT_TRUE(wait_for_reason("none"));
  EXPECT_DOUBLE_EQ(last_command().linear.x, before);
  EXPECT_DOUBLE_EQ(last_command().linear.x, 0.25);
}

TEST_F(CollisionPredictorFixture, TheSignSurvivesTheLimit)
{
  start();
  ASSERT_TRUE(enable_output(true));
  // Facing -x and reversing moves the body towards +x, so the wall it brakes for is ahead of it
  // there. Placed so the braking law cuts the command without reaching zero.
  broadcast_robot(1.0, 1.0, 3.14159265358979);
  ASSERT_TRUE(drive(-0.3, 0.0, 27, 10));

  ASSERT_TRUE(wait_until("a limited reverse command", [this]() {
    const double linear = last_command().linear.x;
    return linear < 0.0 && linear > -0.3;
  }));
  EXPECT_LT(last_command().linear.x, 0.0);
  EXPECT_EQ(value_of(last_diagnostic(), "has_collision"), "true");
}

TEST_F(CollisionPredictorFixture, PureRotationIsZeroedByAPredictedCollision)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.0, 0.5, 20, 10));

  ASSERT_TRUE(wait_for_reason("limited"));
  EXPECT_EQ(value_of(last_diagnostic(), "has_collision"), "true");
  EXPECT_DOUBLE_EQ(last_command().angular.z, 0.0);
  EXPECT_DOUBLE_EQ(last_command().linear.x, 0.0);
}

TEST_F(CollisionPredictorFixture, ClearingTheObstacleRestoresTheCommand)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, 28, 10));
  ASSERT_TRUE(wait_for_reason("limited"));

  // The proximity ramp climbs back over release_time, so this needs more than one cycle.
  ASSERT_TRUE(drive(0.25, 0.0, -1, 40));
  EXPECT_TRUE(wait_until("the request to pass through again", [this]() {
    return value_of(last_diagnostic(), "reason") == "none" && last_command().linear.x == 0.25;
  }));
}

TEST_F(CollisionPredictorFixture, APredictionThatLeavesTheMapZeroesTheCommand)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(3.95, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));

  ASSERT_TRUE(wait_for_reason("outside_map"));
  EXPECT_EQ(value_of(last_diagnostic(), "prediction_truncated"), "true");
  clear();
  ASSERT_TRUE(wait_for_commands(3));
  EXPECT_TRUE(every_command_is_zero());
}

TEST_F(CollisionPredictorFixture, DisablingPublishesOneZeroAndThenStops)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));
  ASSERT_TRUE(wait_until("a non-zero command", [this]() { return peak_linear() > 0.0; }));

  clear();
  ASSERT_TRUE(enable_output(false));
  ASSERT_TRUE(wait_for_commands(1));
  EXPECT_TRUE(every_command_is_zero());

  const std::size_t after_the_zero = command_count();
  ASSERT_TRUE(wait_for_diagnostics(5));
  EXPECT_EQ(command_count(), after_the_zero);
  EXPECT_EQ(value_of(last_diagnostic(), "reason"), "output_disabled");
}

TEST_F(CollisionPredictorFixture, AMapInTheWrongFrameIsRejected)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(publish_command(make_command(helper_->now(), 0.25, 0.0)));
  ASSERT_TRUE(publish_map(make_map(helper_->now(), -1, "odom")));

  ASSERT_TRUE(wait_for_reason("map_rejected"));
  EXPECT_EQ(status_of(last_diagnostic()).level, Status::ERROR);
  EXPECT_NE(status_of(last_diagnostic()).message.find("odom"), std::string::npos);
}

TEST_F(CollisionPredictorFixture, ACommandInTheWrongFrameIsRejected)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(publish_map(make_map(helper_->now())));
  ASSERT_TRUE(publish_command(make_command(helper_->now(), 0.25, 0.0, "odom")));

  ASSERT_TRUE(wait_for_reason("command_rejected"));
  EXPECT_EQ(status_of(last_diagnostic()).level, Status::ERROR);
  clear();
  ASSERT_TRUE(wait_for_commands(3));
  EXPECT_TRUE(every_command_is_zero());
}

TEST_F(CollisionPredictorFixture, TheDiagnosticIsPublishedEveryCycle)
{
  start();
  ASSERT_TRUE(wait_for_diagnostics(10));

  EXPECT_EQ(command_count(), 0u);
  EXPECT_EQ(status_of(last_diagnostic()).name, std::string("/") + NODE_NAME);
  EXPECT_GT(number_of(last_diagnostic(), "cycle_dt"), 0.0);
}

TEST_F(CollisionPredictorFixture, PredictedPosesAndFootprintArePublished)
{
  start();
  ASSERT_TRUE(enable_output(true));
  broadcast_robot(1.0, 1.0, 0.0);
  ASSERT_TRUE(drive(0.25, 0.0, -1, 8));
  ASSERT_TRUE(wait_for_reason("none"));

  const nav_msgs::msg::Path path = last_path();
  EXPECT_EQ(path.header.frame_id, MAP_FRAME);
  EXPECT_EQ(path.poses.size(), 11u);

  const geometry_msgs::msg::PolygonStamped footprint = last_footprint();
  EXPECT_EQ(footprint.header.frame_id, BASE_FRAME);
  EXPECT_EQ(footprint.polygon.points.size(), 4u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
