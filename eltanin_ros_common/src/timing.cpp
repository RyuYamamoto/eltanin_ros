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

#include "eltanin_ros_common/timing.hpp"

#include <cassert>
#include <cstdint>
#include <utility>

namespace eltanin_ros_common
{

PeriodicClock::PeriodicClock(rclcpp::Clock::SharedPtr clock) : clock_(std::move(clock))
{
  assert(clock_ != nullptr);
}

PeriodicClock::Tick PeriodicClock::tick()
{
  // The constructor's assert is gone under RelWithDebInfo, so a null clock skips the cycle instead.
  if (clock_ == nullptr) {
    return Tick{};
  }

  const std::int64_t now_ns = clock_->now().nanoseconds();

  Tick result;
  if (previous_ns_.has_value()) {
    result.seconds = static_cast<double>(now_ns - *previous_ns_) * 1e-9;
    result.has_previous = true;
  }
  // Advanced whatever the verdict was, so the next interval never spans a skipped cycle.
  previous_ns_ = now_ns;
  return result;
}

void PeriodicClock::reset() noexcept
{
  previous_ns_.reset();
}

}  // namespace eltanin_ros_common
