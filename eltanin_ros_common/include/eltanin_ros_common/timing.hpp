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

#ifndef ELTANIN_ROS_COMMON__TIMING_HPP_
#define ELTANIN_ROS_COMMON__TIMING_HPP_

#include <rclcpp/clock.hpp>

#include <cstdint>
#include <optional>

namespace eltanin_ros_common
{

/// Measures the real interval between two calls; it takes no nominal period and keeps none (C-8).
class PeriodicClock
{
public:
  struct Tick
  {
    /// Measured interval [s], never clamped: it is zero when the clock stalls, negative on a jump.
    double seconds{0.0};

    /// False on the first tick after construction or reset(), where seconds carries no meaning.
    bool has_previous{false};

    /// The single predicate for "this interval may be integrated with".
    bool usable() const noexcept { return has_previous && seconds > 0.0; }
  };

  /// Takes the node's clock, so use_sim_time applies without this class knowing about it (NF-4).
  explicit PeriodicClock(rclcpp::Clock::SharedPtr clock);

  /// Reads the clock once and advances the reference time, even when the interval is not usable.
  Tick tick();

  /// Makes the next tick a first tick; for dropping integrated state on a new goal.
  void reset() noexcept;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::optional<std::int64_t> previous_ns_{};
};

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__TIMING_HPP_
