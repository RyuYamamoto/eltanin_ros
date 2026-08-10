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

#ifndef ELTANIN_CONTROLLER__LIMITER_DIAGNOSTIC_HPP_
#define ELTANIN_CONTROLLER__LIMITER_DIAGNOSTIC_HPP_

#include <array>
#include <cstdint>

namespace eltanin_controller
{

/// Why /cmd_vel is what it is; the declaration order is the order run_cycle() decides in.
enum class LimiterReason {
  OutputDisabled,   ///< the service has not enabled the output, so nothing is published (D-22)
  CommandRejected,  ///< cmd_vel_raw arrived in the wrong frame or carried a non-finite value
  CommandMissing,   ///< no cmd_vel_raw has ever arrived
  CommandStale,     ///< cmd_vel_raw is past cmd_timeout, a stamp in the future included
  MapRejected,      ///< the local map arrived in the wrong frame or could not be converted
  MapMissing,       ///< no local map has ever arrived
  MapStale,         ///< the local map is past map_timeout
  NoTransform,      ///< frames.map to frames.base could not be looked up
  OutsideMap,       ///< the rollout left the map, so the command was never checked
  Limited,          ///< the limiter cut the requested command down
  None              ///< the requested command passed through untouched
};

/// Every reason in decision order; the diagnostic vocabulary and the tests both read it from here.
inline constexpr std::array<LimiterReason, 11> LIMITER_REASONS{
  LimiterReason::OutputDisabled,
  LimiterReason::CommandRejected,
  LimiterReason::CommandMissing,
  LimiterReason::CommandStale,
  LimiterReason::MapRejected,
  LimiterReason::MapMissing,
  LimiterReason::MapStale,
  LimiterReason::NoTransform,
  LimiterReason::OutsideMap,
  LimiterReason::Limited,
  LimiterReason::None};

const char * name_of(LimiterReason reason) noexcept;

/// DiagnosticStatus level; ERROR is the wiring being wrong, STALE an input, WARN a passing state.
std::uint8_t level_of(LimiterReason reason) noexcept;

/// True for every reason that forces a zero command; only the last two carry the limiter result.
bool forces_zero_command(LimiterReason reason) noexcept;

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__LIMITER_DIAGNOSTIC_HPP_
