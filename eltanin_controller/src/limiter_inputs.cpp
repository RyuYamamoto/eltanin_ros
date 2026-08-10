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

#include "eltanin_controller/limiter_inputs.hpp"

#include <optional>
#include <utility>

namespace eltanin_controller
{

namespace
{

/// The three reasons an input can fail, in the order the watchdog decides them.
struct InputReasons
{
  LimiterReason rejected;
  LimiterReason missing;
  LimiterReason stale;
};

template <class T>
InputReading<T> read_input(
  const eltanin_ros_common::StaleInput<T> & input, const std::string & rejection,
  const InputReasons & reasons, const rclcpp::Time & now)
{
  InputReading<T> reading;
  const std::optional<double> age = input.elapsed_seconds(now);
  reading.has_age = age.has_value();
  reading.age_seconds = age.value_or(0.0);

  if (!rejection.empty()) {
    reading.reason = reasons.rejected;
    return reading;
  }
  const T * value = input.get(now);
  if (value != nullptr) {
    reading.value = *value;
    return reading;
  }
  reading.reason = reading.has_age ? reasons.stale : reasons.missing;
  return reading;
}

constexpr InputReasons COMMAND_REASONS{
  LimiterReason::CommandRejected, LimiterReason::CommandMissing, LimiterReason::CommandStale};

constexpr InputReasons MAP_REASONS{
  LimiterReason::MapRejected, LimiterReason::MapMissing, LimiterReason::MapStale};

}  // namespace

LimiterInputs::LimiterInputs(double cmd_timeout, double map_timeout)
: command_(cmd_timeout), map_(map_timeout)
{
}

void LimiterInputs::accept_command(const eltanin::Twist2D & command, const rclcpp::Time & stamp)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  command_rejection_.clear();
  command_.update(command, stamp);
}

void LimiterInputs::reject_command(std::string detail)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  command_rejection_ = std::move(detail);
  command_.clear();
}

void LimiterInputs::accept_map(SharedDistanceMap map, const rclcpp::Time & stamp)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  map_rejection_.clear();
  map_.update(std::move(map), stamp);
}

void LimiterInputs::reject_map(std::string detail)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  map_rejection_ = std::move(detail);
  map_.clear();
}

CommandReading LimiterInputs::read_command(const rclcpp::Time & now) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return read_input(command_, command_rejection_, COMMAND_REASONS, now);
}

MapReading LimiterInputs::read_map(const rclcpp::Time & now) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return read_input(map_, map_rejection_, MAP_REASONS, now);
}

std::string LimiterInputs::command_rejection() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return command_rejection_;
}

std::string LimiterInputs::map_rejection() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return map_rejection_;
}

}  // namespace eltanin_controller
