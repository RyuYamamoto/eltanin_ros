# eltanin_ros

ROS 2 packages that connect [`eltanin`](https://github.com/RyuYamamoto/eltanin) — a ROS-independent
C++20 2D navigation library — to ROS 2, and replace the [`navyu`](https://github.com/RyuYamamoto/navyu)
navigation stack. All planning, control and costmap logic lives in `eltanin`; the packages here own
only the ROS boundary: parameters, topics, TF, clocks and lifecycle.

The design is recorded in [`docs/design/eltaninnavyuros.md`](docs/design/eltaninnavyuros.md).

## Safety notice

> **TODO (task 23).** The real-robot (kachaka) startup sequence and its safety warnings are written
> here once the sequence has been validated on hardware. The key warning to document is that
> **enabling command output while the robot is on its dock causes it to drive forward** — the
> `collision_predictor` output must stay disabled until a human has confirmed the robot is off the
> dock (design §7.1 / F-17 / R-5).
>
> Until then, treat every launch file in this repository as simulation-only.

## Workspace layout

`eltanin_ros` is one repository inside a colcon workspace that also holds `eltanin` itself:

```
~/workspace/eltanin_ws/
└── src/
    ├── eltanin/       ROS-independent navigation library (plain CMake)
    ├── eltanin_ros/   this repository
    └── navyu/         the stack being replaced (reference only)
```

`eltanin_vendor` builds and installs `eltanin` through `ExternalProject`, and its default source
path is `${CMAKE_CURRENT_SOURCE_DIR}/../../eltanin`, which resolves to `src/eltanin` from
`src/eltanin_ros/eltanin_vendor`. CI reproduces the same relative layout by checking this
repository out into `src/eltanin_ros` and running `vcs import src < src/eltanin_ros/.repos`.

`eltanin` itself never gains a `package.xml` or any other ROS file.

**Do not run `vcs import` on a machine where `src/eltanin` is your own working copy** — it moves
`HEAD`. Import is for CI and for fresh checkouts.

## Requirements

| Item | Version / note |
|---|---|
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| C++ | C++20 (required by `eltanin`) |
| CMake | 3.20 or later; 3.24 for `eltanin_vendor` (see below) |
| Eigen | 3.4 or later |
| yaml-cpp | any version packaged for Jazzy |
| RMW | `rmw_zenoh_cpp` |

`RMW_IMPLEMENTATION=rmw_zenoh_cpp` is used for every node (design §7 / F-12). `ROS_DOMAIN_ID` and
`ROS_LOCALHOST_ONLY` are not relied on as isolation mechanisms.

`rmw_zenoh_cpp` is **not** part of `/opt/ros/jazzy` on the development machine; it is built from
source in `~/workspace/ros2_ws`. Source that overlay in addition to the distribution:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

## Build

```bash
cd ~/workspace/eltanin_ws
vcs import src < src/eltanin_ros/.repos   # not on a machine where src/eltanin is your working copy
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy
colcon build --symlink-install --packages-ignore eltanin \
  --cmake-args --no-warn-unused-cli -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`RelWithDebInfo` is the default because `eltanin` uses `assert()` for its preconditions, and an
`abort()` on a real robot means the node dies and `/cmd_vel` stops (design §4.3). To build `eltanin`
with those asserts enabled during development, add:

```bash
colcon build --symlink-install --packages-ignore eltanin \
  --cmake-args --no-warn-unused-cli \
  -DCMAKE_BUILD_TYPE=Debug -DELTANIN_VENDOR_BUILD_TYPE=Debug
```

`ELTANIN_VENDOR_BUILD_TYPE` controls only the `eltanin` ExternalProject, so the two build types can
be chosen independently. CI runs both `Debug` and `RelWithDebInfo` (R-18).

### `--packages-ignore eltanin` is not optional

colcon discovers `src/eltanin` as a plain `cmake` package even though it has no `package.xml`, so
without this flag it builds `eltanin` a second time, alongside `eltanin_vendor`. Pass the flag to
`colcon test` too. Three things go wrong otherwise:

- A second prefix `install/eltanin` appears, built **without** `-fPIC`. `find_package(eltanin)` can
  resolve to it, and which prefix wins depends on the order of `CMAKE_PREFIX_PATH` — so a node links
  fine today and fails, or misbehaves, after an unrelated change.
- `eltanin`'s own tests are enabled (they default to `ON` when `eltanin` is the top-level project)
  and require GTest, which need not be present.
- `eltanin`'s 432 tests join `colcon test`, duplicating `eltanin`'s own CI.

Symptoms that the flag was forgotten: `install/eltanin/` exists, or `eltanin`'s tests are being
compiled. Delete `build/eltanin install/eltanin` and rebuild.

### `eltanin_vendor` options

| Cache variable | Default | Meaning |
|---|---|---|
| `ELTANIN_VENDOR_SOURCE_DIR` | `../../eltanin` from the package | Source tree to build, used when `ELTANIN_VENDOR_GIT_URL` is empty |
| `ELTANIN_VENDOR_GIT_URL` | empty | Clone from here instead; requires `ELTANIN_VENDOR_GIT_TAG` |
| `ELTANIN_VENDOR_GIT_TAG` | empty | Revision for `ELTANIN_VENDOR_GIT_URL` |
| `ELTANIN_VENDOR_BUILD_TYPE` | `RelWithDebInfo` | `CMAKE_BUILD_TYPE` for the `eltanin` sub-build only |

The sub-build always gets `-DCMAKE_POSITION_INDEPENDENT_CODE=ON`. `eltanin` declares every module
`STATIC` and does not set that variable itself, while every node here is an `rclcpp_components`
shared library (design §3.3). Without the flag, linking a module into a shared object fails with an
`R_X86_64_PC32` relocation error — four of the eight modules at `-O0`, two of them under
`RelWithDebInfo` — and the rest link by accident. Which ones fail depends on the build type, so the
breakage would surface only once a later node happened to use one of them, or once someone changed
the optimization level. `eltanin_vendor/test/link_check/` builds one `SHARED` library per module with
`WHOLE_ARCHIVE` and `-Wl,--no-undefined` to keep that fixed; it needs CMake 3.24 for the
`WHOLE_ARCHIVE` link feature, which is why the requirements table above names 3.24 for this package.
Because it is part of the build rather than a test, a module that cannot be linked turns
`colcon build` red. Downstream packages do not need to repeat the check.

`BUILD_ALWAYS ON` makes the sub-build re-run every time, so an edit in `src/eltanin` is never missed.
Measured on the development machine: 11.9 s clean, 0.9 s for a no-op rebuild.

`ExternalProject` never removes what it installed. After deleting or renaming a header in `eltanin`,
the stale copy stays in `install/eltanin_vendor/include` and consumers keep including it; run
`rm -rf build/eltanin_vendor install/eltanin_vendor` to clear it.

## Packages

Ten packages are planned (design §4). Everything not yet written is listed with the task that
creates it, so that nothing looks available before it is.

| Package | Contents | Status |
|---|---|---|
| `eltanin_ros` | metapackage (`ament_package` only) | **implemented** |
| `eltanin_vendor` | builds and installs `eltanin` via `ExternalProject` | **implemented** |
| `eltanin_msgs` | msg / action definitions, interfaces only | **implemented** |
| `eltanin_ros_common` | conversions, clock, watchdog, TF, parameter validation | **conversions, clock, watchdog and robot profile implemented**; TF helper waits for its first user |
| `eltanin_costmap` | `global_costmap` and `local_map` nodes | **`global_costmap` implemented**; `local_map` in task 15 |
| `eltanin_planner` | `global_path_planner` and `local_path_planner` nodes | not implemented (tasks 7, 17, 19) |
| `eltanin_controller` | `path_follower` and `collision_predictor` nodes | not implemented (tasks 11, 12) |
| `eltanin_navigator` | orchestrator | not implemented (tasks 13, 21) |
| `eltanin_simulator` | `simple_simulator` node | not implemented (task 10) |
| `eltanin_bringup` | launch / config / rviz / map, no code | not implemented (tasks 8, 22) |

The metapackage's `package.xml` lists one `exec_depend` per implemented package, so the list is also
the list of what exists.

## `eltanin_ros_common`

The package has no nodes and builds two library targets. Which one a header belongs to is readable
from the header itself: one that includes `<rclcpp/...>` is a runtime piece.

| Target | Headers | `rclcpp` |
|---|---|---|
| `eltanin_ros_common` | `conversion_result.hpp`, `cost_conversion.hpp`, `geometry_conversion.hpp`, `map_conversion.hpp`, `path_conversion.hpp`, `scan_conversion.hpp`, `warn_once.hpp` | no |
| `eltanin_ros_common_runtime` | `stale_input.hpp`, `timing.hpp`, `robot_profile.hpp` | yes |

Both are installed through one export set, so a downstream package gets them from a single
`find_package(eltanin_ros_common)`. The runtime target links the conversion layer publicly, so a node
that links the runtime target gets both; **the dependency never runs the other way.** The conversion
layer's `ldd` has no `librclcpp` in it and that is checked, not assumed. Only the tests that
construct a node call `rclcpp::init()`, and today that is one file, `test_robot_profile.cpp`.

### The conversion layer

Everything that crosses between `eltanin`'s types and ROS 2 messages is converted here and nowhere
else: cost value ranges, quaternions, twists, transforms, scans, maps and paths.

Conversions never throw. A conversion that can fail returns `ConversionResult<T>`, a check that
produces no value returns `ConversionStatus`, and a rejection carries **one line** naming the frame,
the resolution, the size, the field that was wrong, its value and the tolerance. Failure is not
logged here — `eltanin_ros_common` has no logger; the caller logs it. For conditions that repeat
every cycle, `WarnOnceLatch::should_warn()` answers whether this is the first occurrence.

`to_costmap_update_msg` is the fifth map conversion and the one that cuts a rectangular patch out of
a costmap. **The rectangle is checked against the costmap rather than trusted**, because a node that
grew it by an inflation radius is exactly where an off-by-one lands; `min > max`, a negative corner
and a corner past the last cell are all rejected with one line naming both the rectangle and the map
size. Both layouts are row major with the same x order, so the patch is a row-wise copy — there is no
transpose and no row reversal anywhere on this path.

### What the conversions do and do not preserve

- **`eltanin::map::Costmap` ⇄ `eltanin_msgs/Costmap` is the only lossless direction.** Raw cost
  values, `float64` resolution, and a round trip that is exact cell for cell.
- **`Costmap` → `nav_msgs/OccupancyGrid` is for visualization only.** The table from
  `costmap_2d` maps 1..252 onto 1..98 with integer arithmetic, so up to three cost values collapse
  onto one occupancy value; 154 of the 252 possible `circumscribed_cost` values cannot be recovered
  from the result. What *is* guaranteed, and pinned by tests, is that the mapping is monotonic and
  that the reserved values stay separated (`255 → -1`, `254 → 100`, `253 → 99`, `0 → 0`, everything
  else inside 1..98), so `Inscribed` is always identifiable. **The design document's claim that the
  three-value classification survives visualization is not achievable and has been replaced by those
  two properties — do not "fix" the table to chase it.**
- **`nav_msgs` narrows the resolution.** `nav_msgs/MapMetaData::resolution` is `float32`, so a map
  that goes through `OccupancyGrid` in either direction comes back with the `float32` value of the
  resolution. Compare resolutions with a tolerance, never with `==`, on that path.
- **A static map is reduced to three values by threshold**, following `nav2_map_server`: every
  negative value and everything between `free_threshold` (25) and `occupied_threshold` (65) becomes
  `NO_INFORMATION`. This is deliberately the conservative direction. Note that
  `eltanin::map_io::load_map` uses different defaults and exclusive bounds for the same job, so the
  same map read from PGM and from `/map` does not give the same costmap; task 6 decides which path
  the stack uses.

### Map origins are checked on three axes, not one

`MapGeometry` has no rotation, so a map whose origin is rotated cannot be represented. Checking only
the yaw is not enough: `tf2::getYaw()` returns 0 for a quaternion that is pure roll or pure pitch,
and 0 for a zero quaternion (`sarg` becomes `NaN`, both comparisons fail, and `atan2(0, 0)` is 0).
An incoming `OccupancyGrid` is therefore rejected unless the origin quaternion is finite, has norm 1
within `1e-3`, and has roll, pitch and yaw all within `1e-6` rad. `eltanin_msgs/Costmap` needs none
of this, because `MapMetaData2D` has no orientation field at all — that absence is the point of the
message, and a test asserts the field does not exist.

A default-constructed `geometry_msgs/Quaternion` is **not** a zero quaternion: the message defines
`float64 w 1`, so it is the identity and is accepted. The zero quaternion that has to be rejected is
one a publisher filled in with zeros itself.

### Two places where `tf2` does not work the way it looks

- `tf2::Quaternion(0, 0, yaw)` **does not exist** in Jazzy. The three-argument constructor was
  removed, and in the old bullet API its argument order was `(yaw, pitch, roll)`, so that call would
  have meant `roll = yaw` — a mistake that compiles. Yaw is turned into a quaternion with
  `setRPY(0, 0, yaw)`; beware that `setEuler` takes yaw *first*.
- `tf2` only **declares** `fromMsg(const geometry_msgs::msg::Quaternion &, tf2::Quaternion &)`; the
  definition lives in `tf2_geometry_msgs`. So `tf2::getYaw(some_geometry_msgs_quaternion)` compiles
  against `tf2` alone but fails to link. `tf2_geometry_msgs` would drag in `tf2_ros` and therefore
  `rclcpp`, so this package builds the `tf2::Quaternion` from the four fields itself and calls
  `tf2::getYaw` / `tf2::getEulerYPR` on that.

### The runtime pieces

Three small types, each shaped so that a check the nodes must not forget cannot be skipped. The
value of each one is the shape, not the code, so these rules are part of the type:

- **`robot.*` and `frames.*` are never declared or read in a node.** `declare_robot_profile(node)`
  is the only path; take the footprint, the radii, the inflation cost model, the velocity limits and
  the frame ids from the `RobotProfile` it returns. It never throws and never logs: check `ok()`,
  and on failure log `error()` and refuse to start. A node holds the profile as a `const
  RobotProfile` member filled in the constructor initializer list — `RobotProfile` has no default
  constructor precisely so that "not validated yet" is not a state a node can be in.
- **`PeriodicClock` does not clamp `dt`, and there is no nominal period in it.** It takes the node's
  clock (`node->get_clock()`, so `use_sim_time` applies) and nothing else. `tick().seconds` is always
  the measurement, including zero and negative; `tick().usable()` is the predicate, and a cycle
  whose tick is not usable is skipped **by the caller**, which is also where an upper bound on `dt`
  belongs if one is wanted.
- **The stamp given to `StaleInput::update()` is `header.stamp` for a sensor input and
  `clock->now()` at reception for something produced inside the stack.** The deadline is checked on
  both sides, so a stamp far in the future is stale too; a clock that jumped recovers on the next
  update. A timeout that is not positive and finite makes every `get()` stale rather than unlimited,
  and `timeout_is_usable()` says so.

`StaleInput::get(now)` returns a pointer that is valid **until the next `update()` and no longer
than the holder**. Do not store it. Combined with the shared-state rule of design §3.3 — one mutex
around one struct, callbacks copy and leave — the holder lives inside that struct, `get()` is called
under the lock, and anything that needs the value **copies it there and releases the lock**.

`declare_robot_profile()` reports **the first violated condition only**, and the order is fixed:
footprint length, then non-finite vertices, then degeneracy, convexity and the origin, then
`inflation_radius` against the circumscribed radius, then `cost_scaling_factor`, the four velocity
limits and the frame ids. Finiteness has to come before the geometry, because `signed_area()` of a
polygon with a `NaN` vertex is not near zero and `contains()` is false everywhere, so a `NaN`
footprint would be reported as a perfectly ordinary non-convex shape. After all of that the three
`create()` calls in `eltanin` are made for real, and a `nullopt` from any of them is a rejection even
though no condition above explains it.

The code defaults are the kachaka measurements, so a node with no parameters at all starts with the
real robot's shape. `robot/kachaka.yaml` (task 22) repeats those numbers and **the yaml is the
authority**; the code defaults are pinned by `test_robot_profile` so a drift is visible. A different
robot must always be given its own profile. Write `robot.footprint` values **with a decimal point**:
`[0, 0, 1, 0]` is an integer array to the parameter server and is rejected — with one line saying
so, not with a crash, but rejected.

`exact_footprint_check` is deliberately **not** a parameter. It is the fix for E-1 and a knob on it
is a knob on whether the robot notices obstacles; `test_exact_footprint_regression` pins both the
default and the behaviour from this side of the vendor boundary.

### Types whose first real user comes later

`WarnOnceLatch` and `StampedScan` have no node using them yet — the tests are their only callers.
They exist because the acceptance conditions they serve cannot be met otherwise: a node must not be
able to invent a `laser_frame`, so the scan conversion returns the frame and stamp together with the
`ScanData` (which carries neither), and a broken 2D assumption must not be silently dropped nor
logged once per cycle. First users are tasks 11, 12 and 15.

The same is true of the three runtime pieces: `RobotProfile` is first consumed by `global_costmap`
(task 6), `StaleInput` and `PeriodicClock` by `path_follower` and `collision_predictor` (tasks 11 and
12), and `StaleInput` again by `local_map` (task 15). If the shape does not fit its first real user,
change it there rather than working around it.

## `eltanin_costmap`

One node so far, `global_costmap`. It owns the whole-area inflated costmap and, with it, the belief a
replan runs against. **Inflation lives here and nowhere else** in the stack.

### It has no timer, and that is the design

There are exactly two triggers: a `/map` message, and a `~/update` call. `LayeredCostmap::update()`
over 4000×4000 = 16 million cells measures **0.285 s at `-O2`**, which no control cycle can absorb.
Replanning happens once every few tens of seconds, so the work is done then and only then.
`local_map/local_map` is subscribed and its observations are accumulated, but **that callback never
calls `update()`**. `~/update` is synchronous: when the response comes back, the whole area has
already been published.

A consequence worth planning for: while `update()` runs it holds the one mutex, so `local_map`
callbacks wait, and with `KeepLast(1)` every window that arrives in that time except the newest is
dropped — at 10 Hz, up to two or three of them per replan. Accumulation is monotone, so a dropped
window costs nothing permanent: the obstacle is seen again in the next one. A deeper queue is *not*
the fix; it would only add latency to windows that are already stale.

### What "an unknown obstacle" means, and why it never goes away by itself

A cell is accumulated when `local_map` reports it `LETHAL_OBSTACLE` **and the static map does not**.
Cells are deduplicated by linear index and kept in discovery order, so the same messages always
produce the same costmap. Since the window comes from `local_map` the points already sit on cell
centres, so the false-`LETHAL` problem of `eltanin/docs/integration-design.md` §7 — a floating-point
cell-boundary slip leaving a lethal cell in free space — cannot arise here. What is stored is the
**global** cell centre, not the local one, so a `local_map` at a different resolution still cannot
round into a neighbouring cell.

**Accumulation is monotone: an obstacle that has gone away stays until `clear_observations`.** A
person who walked through keeps blocking that cell, and a replan will route around it. That is the
design, not an oversight; deciding when to reset is the `navigator`'s job (tasks 13 and 21).
`clear_observations` does **not** publish — call `~/update` after it to see the result.

### Why accumulating state does not break the origin rule

`eltanin/docs/costmap-design.md` §14.2 rests "an origin change does not shift the cells" on there
being no layer that carries state across an update. Accumulating observations breaks that premise in
form. It does not break it here: **this costmap keeps the geometry of the static map for its whole
life and nothing calls `set_origin()` or `center_on()`** — a grep for either in `eltanin_costmap`
returns only the comment saying so. A stored cell index therefore means the same cell on every
update. A second `/map` message does not move the geometry either; it **replaces the whole object and
drops every accumulated cell**, because those indices would otherwise mean cells in a different grid.
Republishing the same map has that same effect, so do not use `/map` as a keep-alive.

### The patch, and the `⊕ r` in it

`~/global_costmap` carries the whole area on every update and is `transient_local`, so a planner that
starts late still gets it. `~/global_costmap_updates` is the same content as a rectangle, and it is an
optimization only: it always travels with a whole-area message under the **same stamp**, so a
consumer that misses a patch recovers on the next whole area. That is why its depth is 1.

The rectangle is the change region — every window received since the last update, plus what
`clear_observations` dropped — **grown by `r = ceil(inflation_radius / resolution)` on every side**
and clamped to the map. The growth is not decoration: the inflation of a cell at the edge of the
change region reaches `r` cells beyond it, and a patch that stopped at the window edge would leave
that ring stale. `r` is the same expression `InflationLayer` uses for its lookup table, and
`test_global_costmap_state` pins the equality by measuring how far the inflation actually reaches.
When nothing changed, there is no patch at all — `publish_rect` returns `std::nullopt`, so "publish
the whole map as a patch" has no spelling.

### Parameters and startup

`inflate_unknown` (`false`), `occupied_threshold` (`65`), `free_threshold` (`25`) and
`publish_visualization` (`true`) are the node's own; everything else comes from
`declare_robot_profile()`. **All four are read once in the constructor and never again**, the same as
`robot.*` and `frames.*`, so `ros2 param set` on them does nothing. A value the node cannot work with
produces one `ERROR` line and a `std::runtime_error` from the constructor: as a component that is a
failed load, and as `ros2 run eltanin_costmap global_costmap` it is an abort after that line — the
two launch paths behave the same way on purpose. `unknown_is_free` is deliberately **not** declared
here; this node builds no traversability model, so it would have no user.

`publish_visualization` exists because `~/global_costmap` is an `eltanin_msgs` type that RViz cannot
draw; `~/global_costmap_visual` is the `OccupancyGrid` for looking at, and it is visualization only.

### Corrections to the design document

Recorded here rather than by editing `docs/design/eltaninnavyuros.md`, same as for the conversions:

- §6.9 says `StaleInput::get(now)` returns `nullopt` when stale. It returns a pointer instead;
  `std::optional<T>` would copy a 16 MB costmap every cycle.
- §6.9 counts nine headers for this package. There are ten public ones: the seven of the conversion
  layer and the three runtime pieces. `outcome_conversion.hpp` is not among them (tasks 20 and 21),
  and `src/diagnostic.hpp` is private and not installed.
- §4.3 (D-25) lists the smoother weights among the `create()` calls that return `nullopt`. They are
  not: `weight_data + 4 * weight_smooth < 2` is an `assert` in `planner::detail`, so it is not
  checked at all under `RelWithDebInfo`. Task 7 validates it at the ROS boundary.
- §6.9 describes the package as `rclcpp`-free. That holds for the conversion layer target only.
- §9 names `global_costmap_node.cpp`. There is no such file and no hand-written `main`:
  `rclcpp_components_register_node(... EXECUTOR MultiThreadedExecutor)` generates the executable, so
  a single-process start and a `component_container_mt` load run the same code and fail the same way.
  A hand-written `main` would make only the single-process path return `1` instead of aborting.
- §6.2 does not say what happens to a `/map` whose `header.frame_id` is not `frames.map`. It is
  rejected, because §10 makes detecting a frame mismatch a completion condition for stage S3.

## Launch

No launch files exist yet. Simulation bringup is added in task 8 and kachaka bringup in task 22.

## Development

### pre-commit

```bash
pipx install pre-commit
pre-commit install
pre-commit run --all-files
```

Run `clang-format` **through pre-commit**, not through the system binary. The hook pins
clang-format 17.0.6 while Ubuntu 24.04 ships 18.1.3, and the pinned version is authoritative:

```bash
pre-commit run clang-format --all-files
```

### Style and lint

- `.clang-format` is byte-identical to `../navyu/.clang-format` (ament_clang_format based, Google
  style, `ColumnLimit: 100`, `PointerAlignment: Middle`, `BreakBeforeBraces: Custom`).
- `CPPLINT.cfg` sits at the repository root and applies to every package. It disables the six
  checks `ament_cpplint` disables unconditionally, plus `build/header_guard` and
  `build/include_order`. Those last two cannot be reconciled with the ROS 2 conventions:
  upstream cpplint cannot produce ROS 2 double-underscore guard names (that is an `ament_cpplint`
  extension), and its include order is the reverse of the `IncludeCategories` in `.clang-format`.
- Because cpplint no longer checks it, the **header guard convention is enforced by review**:
  `#ifndef PACKAGE__PATH_TO_FILE_HPP_`, e.g. `ELTANIN_ROS_COMMON__CONVERSIONS_HPP_`.
- `legal/copyright` stays enabled. Every source file starts with `// Copyright <year> RyuYamamoto.`
  followed by the Apache-2.0 header, matching `eltanin` and `navyu`.
- A local hook, `tools/check_cost_literals.py`, fails if `253`, `254`, `255`, `97` or `251` appear in
  any package's `include/` or `src/` outside `cost_conversion.{hpp,cpp}`. That is the fence around
  D-20 (the cost range is converted in one place). Only those five numbers are checked: `100`, `-1`,
  `65` and `25` occur too often in ordinary code, and a hook with false positives gets disabled.
  Tests are exempt — writing the table's values down is what a test is for. The hook is a fence, not
  a proof; nothing stops a second implementation that spells the values differently.
- A second local hook, `tools/check_robot_params.py`, fails if `declare_parameter("robot.` or
  `declare_parameter("frames.` appears outside `robot_profile.{hpp,cpp}`. It also catches
  `get_parameter`, which the acceptance condition did not ask for: reading those keys outside
  `robot_profile` skips the validation just as effectively as declaring them, and there is no honest
  reason to do it. Tests are exempt, and it is a fence rather than a proof for the same reason as
  the one above.

### Compiler warnings

Each package adds `-Wall -Wextra -Wpedantic` with
`target_compile_options(<target> PRIVATE ...)` and promotes them to errors when
`ELTANIN_ROS_ENABLE_WERROR` is `ON`. CI passes `-DELTANIN_ROS_ENABLE_WERROR=ON`. These flags are
never set through `CMAKE_CXX_FLAGS`, because that would leak them into the `eltanin_vendor`
ExternalProject and into `rosidl` generated code.

### API rules

`eltanin`'s preconditions are `assert()`-based and disappear under `RelWithDebInfo`, so an
out-of-range cell access becomes undefined behaviour rather than an abort (R-18).

`ELTANIN_VENDOR_BUILD_TYPE` does not fully control this. A large part of `eltanin`'s precondition
checking lives in headers — `GridMap::operator()`, `MapGeometry::index`, the collision predicates —
and an `assert` in a header obeys the **consuming** translation unit's `NDEBUG`, which comes from
`CMAKE_BUILD_TYPE`. `ELTANIN_VENDOR_BUILD_TYPE` governs only the checks inside `eltanin`'s `.cpp`
files. CI passes the same value to both, but a mixed build (`-DCMAKE_BUILD_TYPE=Debug
-DELTANIN_VENDOR_BUILD_TYPE=RelWithDebInfo`) leaves only the header-side asserts live. **The rule
below is therefore required at every build type, not just the default one.**

**Do not write raw `GridMap::operator()` in `eltanin_ros`.** Use the bounds-checked API only:
`get` / `set` / `world_to_map` / `world_rect_to_cells`. Validate `resolution() > 0`,
`cell_count() > 0`, `create()` returning `nullopt`, and `dt > 0` at the ROS boundary and reject bad
input with a single error line naming the offending parameter (design §4.3).

**Do not convert between `eltanin` types and messages inside a node.** Call
`eltanin_ros_common`; if the conversion you need is missing, add it there. Cell copies in that
package go through `GridMap::data()` in one pass, which is why no `operator()` appears in it: the
row-major layout of `MapGeometry`, `OccupancyGrid` and `eltanin_msgs/Costmap` is identical, so no row
flip or transpose is involved. (`eltanin::map_io::load_map` does flip rows, because PGM starts at the
top; that does not apply to messages.)

### Branches

Work happens on `dev`. `main` is updated only through a pull request from `dev` (D-28); never push
to `main` directly.

## License

Apache-2.0, matching `eltanin` and `navyu`. A `LICENSE` file is not yet committed to this
repository; source files carry the Apache-2.0 header.
