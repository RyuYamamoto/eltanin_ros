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

#include "eltanin_costmap/global_costmap.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eltanin/map/cost_values.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using eltanin::map::LETHAL_OBSTACLE;
using eltanin_costmap::GlobalCostmap;
using Trigger = std_srvs::srv::Trigger;

constexpr double RESOLUTION = 0.05;
constexpr std::uint32_t WIDTH = 4;
constexpr std::uint32_t HEIGHT = 3;

/// Long enough that a loaded CI machine still gets there, short enough to fail rather than hang.
constexpr std::chrono::seconds DEADLINE{10};
constexpr std::chrono::milliseconds POLL_INTERVAL{2};

nav_msgs::msg::OccupancyGrid make_grid(const std::string & frame_id = "map")
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.info.resolution = RESOLUTION;
  msg.info.width = WIDTH;
  msg.info.height = HEIGHT;
  msg.info.origin.position.x = -0.1;
  msg.info.origin.position.y = 0.2;
  msg.data.assign(static_cast<std::size_t>(WIDTH) * HEIGHT, 0);
  msg.data[0] = 100;
  return msg;
}

/// The shipped configuration, so a key missing from it fails here instead of on the robot.
std::vector<rclcpp::Parameter> shipped_configuration()
{
  std::vector<rclcpp::Parameter> parameters;
  const std::vector<std::string> files{
    ament_index_cpp::get_package_share_directory("eltanin_ros_common") +
      "/config/robot/kachaka.yaml",
    ament_index_cpp::get_package_share_directory("eltanin_costmap") +
      "/config/global_costmap.param.yaml"};
  for (const std::string & file : files) {
    for (const auto & [node_name, values] : rclcpp::parameter_map_from_yaml_file(file)) {
      (void)node_name;
      parameters.insert(parameters.end(), values.begin(), values.end());
    }
  }
  return parameters;
}

/// Both nodes share one MultiThreadedExecutor, the executor the generated entry point uses.
class GlobalCostmapFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    map_publisher_ = helper_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
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
    std::vector<rclcpp::Parameter> merged = shipped_configuration();
    merged.insert(merged.end(), parameters.begin(), parameters.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(merged);
    node_ = std::make_shared<GlobalCostmap>(options);
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

  ::testing::AssertionResult publish_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    const auto matched = wait_until("the node subscribes to /map", [this]() {
      return map_publisher_->get_subscription_count() > 0;
    });
    if (!matched) {
      return matched;
    }
    map_publisher_->publish(msg);
    return ::testing::AssertionSuccess();
  }

  /// Calls a Trigger service and returns the response, or nullptr when it never answered.
  Trigger::Response::SharedPtr call(const std::string & name)
  {
    const auto client = helper_->create_client<Trigger>(name);
    if (!client->wait_for_service(DEADLINE)) {
      return nullptr;
    }
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (future.wait_for(DEADLINE) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<GlobalCostmap> node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spinner_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
};

TEST_F(GlobalCostmapFixture, StartsFromTheShippedConfigurationAndSurvivesWithoutAMap)
{
  ASSERT_NO_THROW(start());
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(GlobalCostmapFixture, RefusesToStartOnThresholdsThatCrossOver)
{
  try {
    start({rclcpp::Parameter("occupied_threshold", 10), rclcpp::Parameter("free_threshold", 50)});
    FAIL() << "the node started with free_threshold above occupied_threshold";
  } catch (const std::runtime_error & error) {
    const std::string line = error.what();
    EXPECT_NE(line.find("free_threshold"), std::string::npos) << line;
    EXPECT_NE(line.find("occupied_threshold"), std::string::npos) << line;
    EXPECT_EQ(line.find('\n'), std::string::npos) << line;
  }
}

TEST_F(GlobalCostmapFixture, RefusesToStartOnAFootprintThatIsNotAShape)
{
  EXPECT_THROW(
    start({rclcpp::Parameter("robot.footprint", std::vector<double>{0.0, 0.0})}),
    std::runtime_error);
}

TEST_F(GlobalCostmapFixture, UpdateFailsWithTheResolvedTopicNameWhileNoMapHasArrived)
{
  start();
  const auto response = call("/global_costmap/update");
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("/map"), std::string::npos) << response->message;
}

TEST_F(GlobalCostmapFixture, ClearingObservationsSucceedsEvenBeforeAMapArrives)
{
  start();
  const auto response = call("/global_costmap/clear_observations");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->message.empty());
}

TEST_F(GlobalCostmapFixture, ASubscriberThatConnectsAfterTheMapStillGetsTheWholeArea)
{
  start();
  ASSERT_TRUE(publish_map(make_grid()));

  eltanin_msgs::msg::Costmap::ConstSharedPtr received;
  const auto subscription = helper_->create_subscription<eltanin_msgs::msg::Costmap>(
    "/global_costmap/global_costmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&received](eltanin_msgs::msg::Costmap::ConstSharedPtr msg) { received = msg; });

  ASSERT_TRUE(wait_until("the whole area arrives", [&received]() { return received != nullptr; }));
  EXPECT_EQ(received->header.frame_id, "map");
  EXPECT_EQ(received->info.width, WIDTH);
  EXPECT_EQ(received->info.height, HEIGHT);
  EXPECT_DOUBLE_EQ(received->info.resolution, static_cast<double>(static_cast<float>(RESOLUTION)));
  EXPECT_DOUBLE_EQ(received->info.origin_x, -0.1);
  EXPECT_DOUBLE_EQ(received->info.origin_y, 0.2);
  EXPECT_EQ(received->data.size(), static_cast<std::size_t>(WIDTH) * HEIGHT);
  EXPECT_EQ(received->data[0], LETHAL_OBSTACLE);
}

TEST_F(GlobalCostmapFixture, TheVisualizationTopicIsAlsoLatchedForALateSubscriber)
{
  start();
  ASSERT_TRUE(publish_map(make_grid()));

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr received;
  const auto subscription = helper_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/global_costmap_visual",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&received](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { received = msg; });

  ASSERT_TRUE(
    wait_until("the visualization arrives", [&received]() { return received != nullptr; }));
  EXPECT_EQ(received->info.width, WIDTH);
  EXPECT_EQ(received->data[0], 100);
}

TEST_F(GlobalCostmapFixture, NoVisualizationTopicExistsWhenItIsTurnedOff)
{
  start({rclcpp::Parameter("publish_visualization", false)});

  // Seeing the whole-area publisher is what proves the node's publishers have been discovered.
  ASSERT_TRUE(wait_until("the whole area publisher is discovered", [this]() {
    return helper_->count_publishers("/global_costmap/global_costmap") > 0;
  }));
  EXPECT_EQ(helper_->count_publishers("/global_costmap/global_costmap_visual"), 0u);
}

TEST_F(GlobalCostmapFixture, TheWholeAreaIsAlreadyPublishedWhenUpdateAnswers)
{
  start();
  ASSERT_TRUE(publish_map(make_grid()));

  std::atomic<int> count{0};
  const auto subscription = helper_->create_subscription<eltanin_msgs::msg::Costmap>(
    "/global_costmap/global_costmap",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable(),
    [&count](eltanin_msgs::msg::Costmap::ConstSharedPtr) { ++count; });
  ASSERT_TRUE(wait_until("the first whole area arrives", [&count]() { return count.load() > 0; }));
  const int before = count.load();

  const auto response = call("/global_costmap/update");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_NE(response->message.find("updated in"), std::string::npos) << response->message;
  EXPECT_NE(response->message.find("no patch"), std::string::npos) << response->message;
  ASSERT_TRUE(
    wait_until("the second whole area arrives", [&count, before]() { return count > before; }));
}

TEST_F(GlobalCostmapFixture, AMapInTheWrongFrameNeverReachesTheStateAtAll)
{
  start();
  ASSERT_TRUE(publish_map(make_grid("odom")));

  const auto response = call("/global_costmap/update");
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success) << response->message;
}

TEST_F(GlobalCostmapFixture, AnObservedObstacleReachesTheWholeAreaAndAPatchAfterUpdate)
{
  start();
  ASSERT_TRUE(publish_map(make_grid()));

  const auto window_publisher = helper_->create_publisher<eltanin_msgs::msg::Costmap>(
    "/local_map/local_map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  ASSERT_TRUE(wait_until("the node subscribes to the window topic", [&window_publisher]() {
    return window_publisher->get_subscription_count() > 0;
  }));

  eltanin_msgs::msg::Costmap window;
  window.header.frame_id = "map";
  window.info.resolution = static_cast<double>(static_cast<float>(RESOLUTION));
  window.info.width = 2;
  window.info.height = 1;
  window.info.origin_x = -0.1 + 2 * RESOLUTION;
  window.info.origin_y = 0.2 + 1 * RESOLUTION;
  window.data = {LETHAL_OBSTACLE, 0};

  eltanin_msgs::msg::CostmapUpdate::ConstSharedPtr patch;
  const auto patch_subscription = helper_->create_subscription<eltanin_msgs::msg::CostmapUpdate>(
    "/global_costmap/global_costmap_updates", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&patch](eltanin_msgs::msg::CostmapUpdate::ConstSharedPtr msg) { patch = msg; });
  ASSERT_TRUE(wait_until("the patch subscription matches", [&patch_subscription]() {
    return patch_subscription->get_publisher_count() > 0;
  }));

  window_publisher->publish(window);
  // The window is absorbed on its own callback, so the only way to see it is to ask for an update.
  Trigger::Response::SharedPtr response;
  ASSERT_TRUE(wait_until("an update reports a patch", [this, &response]() {
    response = call("/global_costmap/update");
    return response != nullptr && response->message.find("no patch") == std::string::npos;
  }));
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_NE(response->message.find("1 observation cells"), std::string::npos) << response->message;

  ASSERT_TRUE(wait_until("the patch arrives", [&patch]() { return patch != nullptr; }));
  EXPECT_EQ(patch->header.frame_id, "map");
  EXPECT_EQ(patch->data.size(), patch->width * patch->height);
  EXPECT_GT(patch->data.size(), 0u);
}

TEST_F(GlobalCostmapFixture, ASecondMapRebuildsAndDropsTheObservations)
{
  start();
  ASSERT_TRUE(publish_map(make_grid()));
  ASSERT_TRUE(wait_until("the first update succeeds", [this]() {
    const auto response = call("/global_costmap/update");
    return response != nullptr && response->success;
  }));

  auto second = make_grid();
  second.info.origin.position.x = 1.0;
  ASSERT_TRUE(publish_map(second));

  eltanin_msgs::msg::Costmap::ConstSharedPtr received;
  const auto subscription = helper_->create_subscription<eltanin_msgs::msg::Costmap>(
    "/global_costmap/global_costmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&received](eltanin_msgs::msg::Costmap::ConstSharedPtr msg) { received = msg; });
  ASSERT_TRUE(wait_until("the rebuilt map arrives", [&received]() {
    return received != nullptr && received->info.origin_x == 1.0;
  }));

  const auto response = call("/global_costmap/update");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_NE(response->message.find("0 observation cells"), std::string::npos) << response->message;
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
