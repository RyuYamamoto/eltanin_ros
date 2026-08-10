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

#ifndef ELTANIN_CONTROLLER__LIMITER_INPUTS_HPP_
#define ELTANIN_CONTROLLER__LIMITER_INPUTS_HPP_

#include "eltanin_controller/limiter_diagnostic.hpp"

#include <eltanin/core/types.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin_ros_common/stale_input.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace eltanin_controller
{

/// The distance map is built once in the subscription; the timer only ever copies this pointer.
using SharedDistanceMap = std::shared_ptr<const eltanin::map::DistanceMap>;

/// One input as the cycle sees it; reason is None exactly when the value is usable.
template <class T>
struct InputReading
{
  LimiterReason reason{LimiterReason::None};
  T value{};
  /// now - stamp [s], negative for a stamp in the future; for the diagnostic, not for the check.
  double age_seconds{0.0};
  /// False before the first message ever arrives, when there is no age to report.
  bool has_age{false};
};

using CommandReading = InputReading<eltanin::Twist2D>;
using MapReading = InputReading<SharedDistanceMap>;

/// The only mutable state the node shares between threads; the subscriptions are its only writers.
class LimiterInputs
{
public:
  LimiterInputs(double cmd_timeout, double map_timeout);

  /// Stores the received command exactly as it arrived; nothing ever writes a limited value back.
  void accept_command(const eltanin::Twist2D & command, const rclcpp::Time & stamp);

  /// Drops the held command: a rejected message means the node no longer knows what was requested.
  void reject_command(std::string detail);

  void accept_map(SharedDistanceMap map, const rclcpp::Time & stamp);

  void reject_map(std::string detail);

  /// Copies the value out under the lock, so the caller never computes while holding it.
  CommandReading read_command(const rclcpp::Time & now) const;

  MapReading read_map(const rclcpp::Time & now) const;

  std::string command_rejection() const;

  std::string map_rejection() const;

private:
  mutable std::mutex mutex_;
  eltanin_ros_common::StaleInput<eltanin::Twist2D> command_;
  eltanin_ros_common::StaleInput<SharedDistanceMap> map_;
  std::string command_rejection_;
  std::string map_rejection_;
};

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__LIMITER_INPUTS_HPP_
