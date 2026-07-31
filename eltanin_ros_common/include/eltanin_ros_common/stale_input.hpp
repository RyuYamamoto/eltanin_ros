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

#ifndef ELTANIN_ROS_COMMON__STALE_INPUT_HPP_
#define ELTANIN_ROS_COMMON__STALE_INPUT_HPP_

#include <rclcpp/time.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

namespace eltanin_ros_common
{

/// Holds one input behind its deadline; get(now) is the only path to the value (N-1 / N-4).
template <class T>
class StaleInput
{
public:
  /// A timeout that is not positive and finite makes every get() stale; it is never "no deadline".
  explicit StaleInput(double timeout_seconds) noexcept
  : timeout_ns_(to_nanoseconds(timeout_seconds)),
    timeout_usable_(std::isfinite(timeout_seconds) && timeout_seconds > 0.0)
  {
  }

  /// Value and stamp are replaced together; there is no way to refresh the stamp alone (A-11).
  void update(T value, const rclcpp::Time & stamp)
  {
    value_ = std::move(value);
    stamp_ns_ = stamp.nanoseconds();
  }

  /// Valid until the next update() and never beyond this object; copy the value out, do not store.
  const T * get(const rclcpp::Time & now) const noexcept
  {
    if (!value_.has_value()) {
      return nullptr;
    }
    if (!timeout_usable_) {
      return nullptr;
    }
    if (distance_ns(now.nanoseconds(), stamp_ns_) > static_cast<std::uint64_t>(timeout_ns_)) {
      return nullptr;
    }
    return &*value_;
  }

  /// now - stamp [s], negative for a stamp in the future; for the diagnostic line, not a check.
  std::optional<double> elapsed_seconds(const rclcpp::Time & now) const noexcept
  {
    if (!value_.has_value()) {
      return std::nullopt;
    }
    return (static_cast<double>(now.nanoseconds()) - static_cast<double>(stamp_ns_)) * 1e-9;
  }

  /// False when the configured timeout was not positive and finite, so get() can never succeed.
  bool timeout_is_usable() const noexcept { return timeout_usable_; }

private:
  static std::int64_t to_nanoseconds(double seconds) noexcept
  {
    constexpr double LIMIT = 9.0e18;
    const double nanoseconds = seconds * 1e9;
    if (!(nanoseconds < LIMIT)) {
      return std::numeric_limits<std::int64_t>::max();
    }
    if (nanoseconds <= 0.0) {
      return 0;
    }
    return static_cast<std::int64_t>(nanoseconds);
  }

  /// Unsigned subtraction, so two stamps from different epochs cannot overflow the difference.
  static std::uint64_t distance_ns(std::int64_t lhs, std::int64_t rhs) noexcept
  {
    const auto left = static_cast<std::uint64_t>(lhs);
    const auto right = static_cast<std::uint64_t>(rhs);
    return lhs >= rhs ? left - right : right - left;
  }

  std::optional<T> value_{};
  std::int64_t stamp_ns_{0};
  std::int64_t timeout_ns_{0};
  bool timeout_usable_{false};
};

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__STALE_INPUT_HPP_
