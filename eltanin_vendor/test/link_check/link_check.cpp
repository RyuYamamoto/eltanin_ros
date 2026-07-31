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

#include <eltanin/collision/velocity_limiter.hpp>

namespace eltanin_vendor
{

// Only here to give the surrounding SHARED library something to compile; the link is the check.
double default_max_deceleration()
{
  const eltanin::collision::VelocityLimiterParams params;
  return params.max_deceleration;
}

}  // namespace eltanin_vendor
