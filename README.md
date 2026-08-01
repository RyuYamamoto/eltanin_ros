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
| `eltanin_ros_common` | conversions, clock, watchdog, TF, parameter validation | **conversions, clock, watchdog and robot profile implemented**; the TF helper stays inside `global_path_planner` until a second user appears |
| `eltanin_costmap` | `global_costmap` and `local_map` nodes | **`global_costmap` implemented**; `local_map` in task 15 |
| `eltanin_planner` | `global_path_planner` and `local_path_planner` nodes | **`global_path_planner` implemented**; `local_path_planner` in tasks 17 and 19 |
| `eltanin_controller` | `path_follower` and `collision_predictor` nodes | not implemented (tasks 11, 12) |
| `eltanin_navigator` | orchestrator | not implemented (tasks 13, 21) |
| `eltanin_simulator` | `simple_simulator` node | not implemented (task 10) |
| `eltanin_bringup` | launch / config / rviz / map, no code | **`eltanin_bringup.launch.py` implemented**; simulation and kachaka bringup in tasks 10 and 22 |

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
  not: `weight_data + 4 * weight_smooth < 2` is enforced by `planner::detail`, which throws
  `std::invalid_argument` from inside `smooth()`. `eltanin_planner` rejects them at startup instead,
  where the report is one line and the caller has somewhere to put it.
- §6.9 describes the package as `rclcpp`-free. That holds for the conversion layer target only.
- §9 names `global_costmap_node.cpp`. There is no such file and no hand-written `main`:
  `rclcpp_components_register_node(... EXECUTOR MultiThreadedExecutor)` generates the executable, so
  a single-process start and a `component_container_mt` load run the same code and fail the same way.
  A hand-written `main` would make only the single-process path return `1` instead of aborting.
- §6.2 does not say what happens to a `/map` whose `header.frame_id` is not `frames.map`. It is
  rejected, because §10 makes detecting a frame mismatch a completion condition for stage S3.

## `eltanin_planner`

`global_path_planner` turns a goal into a path over the belief `global_costmap` publishes. It exists
because navyu could not say why a plan failed (N-5): `eltanin::planner::plan()` collapses four
distinct causes into one `nullopt`, and this node is where they are told apart again.

### Why the action, and what its outcomes mean

Planning is a request/response shape, but it is an **action** rather than a service: a search is
0.147 s at `-O2` and 1.48 s at `-O0` on a 4000x4000 map, which is long enough for a cancellation to
be worth having (§3.2).

`~/compute_path_to_pose` (`eltanin_msgs/ComputePathToPose`) takes `goal`, `start` and `use_start`,
and returns `path`, `outcome` and `message`. **Feedback is never sent** — the action's feedback is
empty, because a single search has no intermediate progress to report. `handle_goal` never inspects
the goal and always accepts: a rejection carries no result, so a rejected goal could not report an
`outcome`, which is the whole point of the node. Every content-related failure is therefore an
accepted goal that is aborted with a reason.

`outcome` uses `eltanin_msgs/NavigationState`'s `OUTCOME_*` values and stays inside six of them:

| `outcome` | When |
|---|---|
| `OUTCOME_REACHED` (0) | A path was produced. In a planner **it means "a path came out", not "the robot arrived"**; the value set has no other spelling for success. |
| `OUTCOME_INPUT_STALE` (13) | No whole-area costmap has arrived yet. |
| `OUTCOME_START_GOAL_FAILED` (2) | An empty or untransformable `frame_id`, an unusable pose, a start or goal outside the map, or a start with no free cell inside `start_search_radius_cells`. |
| `OUTCOME_NO_PATH` (5) | The goal cell is not `Free`. **The goal is reported, never moved** — `find_nearest_traversable` is applied to the start only. |
| `OUTCOME_PLAN_FAILED` (3) | The search itself found nothing, or returned no poses. |
| `OUTCOME_CANCELED` (11) | The goal was canceled, or preempted by a newer one. There is no separate value for a preemption; `message` says which it was. |

`message` is filled on success and on failure, and it is the same single line the node logs.

### Telling the four causes apart costs a deliberate duplication

`attempt_plan()` runs the checks `Planner::plan()` runs — `world_to_map`, `classify(goal)`,
`find_nearest_traversable(start, r)` — **in the same order, with the same radius and the same model**,
before calling `plan()` itself. That duplication is intentional: giving `eltanin`'s `plan()` a
reason-carrying return value would change its API for the sake of the ROS boundary alone, and
`eltanin` is not modified from here. The consequence is a coupling worth knowing about: **if
`eltanin`'s `plan()` ever changes its checks, `attempt_plan()` has to follow**, or a real search
failure will be reported as one of the endpoint failures. `test_goal_validation.cpp` pins each
boundary, and the smoothing step is deliberately left outside `attempt_plan()` so that a
cancellation can be observed between the search and the smoother.

### Two searches, one set of failure classes

`planner_type` selects between `astar` (8-connected, over cells) and `hybrid_astar` (over
`(x, y, yaw)`, honouring `hybrid.minimum_turning_radius`). Both go through `eltanin`'s same
`Planner::plan()`, so the pre-checks and the six outcomes above are shared; only the search itself
differs, and the failing line names which one ran.

**A Hybrid A\* path is published unsmoothed.** The smoother only pulls interior points towards the
average of their neighbours: it would take back exactly the curvature bound Hybrid A\* was asked to
respect. `smoother_*` therefore has no effect while `planner_type` is `hybrid_astar`, and
`~/global_path_raw` carries the same path as `~/global_path`.

Hybrid A\* checks traversability at the vehicle reference point only, which is sound here because
the costmap it plans on is the inflated one `global_costmap` publishes. Turn on
`publish_footprint_path` to see that for yourself.

**`hybrid.max_states` is a memory fence, and it is not optional.** The search sizes three arrays
from `cells * heading_bins * <its own motion mode count>` *before* it expands anything, so
`max_expansions` cannot bound it. On the 4000x4000 map at 0.05 m with the default 72 heading
bins that is 4.6 billion states: measured at 74 GB of virtual and 55 GB resident, which took the
development machine's OOM killer to stop. Above `hybrid.max_states` the plan is refused with one
line and `OUTCOME_PLAN_FAILED` instead. Raise `hybrid.max_states` only together with a smaller
map or fewer `heading_bins`, and budget roughly 70 bytes per state.

### Interruption is answered at four points, not during the search

A search cannot be interrupted, so a cancel or a preemption is observed only at four boundaries:
after acceptance, after the endpoints are resolved, after the search, and after the smoother. **A
cancel is therefore answered at worst one whole search late** — 0.147 s at `-O2`, 1.48 s at `-O0`.
A newer goal always wins: the running goal is aborted with `OUTCOME_CANCELED`, and a goal still
waiting is displaced the same way, since only one plan runs at a time and there is no queue. Planning
runs on a single worker thread rather than in the executor, so "at most one plan at a time" is a
property of this code and not of `std::thread::hardware_concurrency()`.

### Topics

| Topic | Type | QoS | Note |
|---|---|---|---|
| `global_costmap/global_costmap` (in) | `eltanin_msgs/Costmap` | `KeepLast(1)`, reliable, `transient_local` | Replaces the belief. Relative name; the launch file remaps it. |
| `global_costmap/global_costmap_updates` (in) | `eltanin_msgs/CostmapUpdate` | `KeepLast(1)`, reliable, volatile | Applied to the belief. |
| `~/global_path` (out) | `nav_msgs/Path` | `KeepLast(1)`, reliable, volatile | Smoothed. Published before `succeed()`, so a successful result always has its path on the topic too. |
| `~/global_path_raw` (out) | `nav_msgs/Path` | same | Unsmoothed; **the publisher only exists when `publish_raw_path` is set**. |
| `~/footprint_path` (out) | `visualization_msgs/MarkerArray` | same | `robot.footprint` laid along the published path, every `footprint_marker_stride` poses and always at the last one. Only exists when `publish_footprint_path` is set. |

Neither path topic is `transient_local`: a late subscriber would otherwise be handed, as the newest
thing available, the path of a goal that was abandoned long ago. The authoritative hand-over is the
action result; the topics are for visualization and for consumers that follow the latest.

**Applying the patch is redundant today** and is implemented anyway. `global_costmap` always sends a
whole area under the same stamp as its patch, so dropping every patch would give the same belief. It
is here so that reducing how often the whole area is published — the open question U-P6-1, and task
24 — needs no change in this node. A patch that is not strictly newer than the held costmap is
dropped, which is what keeps the same-stamp pair from being applied twice.

There is no staleness deadline on the costmap. `global_costmap` publishes on `/map` and on `~/update`
only, never from a timer, so "nothing arrived recently" is the normal state of a standing robot; the
node only asks whether one ever arrived.

### The goal is not trusted

navyu assumed every goal was already in the map frame (§2.4-2). Here `header.frame_id` is checked:
empty is a rejection rather than an assumption, a frame equal to `frames.map` skips tf entirely, and
anything else is transformed or rejected. The failing line names both frames, the stamp it looked
up, and the timeout it waited — because that is the information needed to fix it. **The requested
`orientation` is not discarded**: `plan()` puts `goal.yaw` on the last pose and the smoother leaves
the last pose's yaw alone, so the yaw that was asked for is the yaw that comes back.

### Parameters and startup

| Key | Default | From |
|---|---|---|
| `planner_type` | `astar` | `astar` or `hybrid_astar` |
| `start_search_radius_cells` | `8` | `eltanin::planner::AStarParams`; both searches take it, so it is one key |
| `hybrid.heading_bins` | `72` | `eltanin::planner::HybridAStarParams` |
| `hybrid.minimum_turning_radius` | `0.4` | same |
| `hybrid.motion_step` | `0.0` | same; 0 selects one map cell |
| `hybrid.collision_check_step` | `0.0` | same; 0 selects half a map cell |
| `hybrid.dubins_expansion_distance` | `1.0` | same |
| `hybrid.steering_penalty` | `0.05` | same |
| `hybrid.steering_change_penalty` | `0.10` | same |
| `hybrid.max_expansions` | `0` | same; 0 is no bound |
| `hybrid.max_states` | `20000000` | this node; a ceiling on `cells * heading_bins` |
| `publish_footprint_path` | `false` | this node |
| `footprint_marker_stride` | `10` | this node |
| `weight_data` | `0.5` | `eltanin::planner::SmootherParams` |
| `weight_smooth` | `0.3` | same |
| `smoother_tolerance` | `1e-4` | same (`tolerance`) |
| `smoother_max_iterations` | `100` | same |
| `publish_raw_path` | `false` | design §6.3 |
| `unknown_is_free` | `false` | the second argument of `CostTraversabilityModel` |
| `tf_lookup_timeout` | `0.1` | this node |

`PlannerParameters` holds `AStarParams` and `SmootherParams` by value rather than copying their
numbers, so the defaults above cannot drift from `eltanin`'s; `test_planner_parameters.cpp` pins them
so that a change on the `eltanin` side is noticed rather than absorbed. As in `eltanin_costmap`, all
eight are read once in the constructor and never again, and a value the node cannot work with
produces one `ERROR` line and a `std::runtime_error` from the constructor. Only the first violated
condition is reported.

**The `hybrid.*` values are validated whichever search is selected**, so switching `planner_type`
later cannot turn a value that was sitting unused into a `std::invalid_argument` from
`HybridAStarPlanner`'s constructor, thrown mid-plan.

`weight_data + 4 * weight_smooth < 2` is checked at startup even though `eltanin` checks it too.
Breaking it makes the smoother diverge, and `eltanin`'s own report is a `std::invalid_argument`
thrown from inside `smooth()` — that is, once per plan request, from the middle of the call stack,
long after the misconfiguration could still be fixed. The same holds for a negative
`start_search_radius_cells`, which `Planner`'s constructor throws on. Validating at startup turns
both into one `ERROR` line and a node that does not come up, and it does not depend on how `eltanin`
was built.

`unknown_is_free` is declared here and not in `eltanin_costmap`, because this node builds the first
`CostTraversabilityModel` in the stack. Its threshold comes from
`profile_.inflation_cost_model().circumscribed_cost()`, the same expression `global_costmap` inflates
with — so **both nodes have to be given the same `robot/*.yaml`**, or the boundary between `Free` and
`Circumscribed` here will not match the inflation there.

### Measured planning time

On a 4000x4000 empty map at `-O2`, `attempt_plan()` from cell (10, 10) to cell (3990, 3990) takes
**96 to 122 ms** over four runs and returns 3981 poses. That is in line with the 0.147 s `eltanin`
records for its own worst case, and it is what the 0.147 s figure in the cancellation discussion
above refers to. The measurement is `AttemptPlanTest.DISABLED_PlanScale`, kept disabled so CI never
runs it at `-O0`:

```bash
./build/eltanin_planner/test_goal_validation \
  --gtest_also_run_disabled_tests --gtest_filter=*PlanScale*
```

### Corrections to the design document

- §9 names `global_path_planner_node.cpp`. There is no such file and no hand-written `main`, for the
  same reason as `global_costmap_node.cpp` above.
- §3.3 says tf lookups use a timeout of 0. `global_path_planner` deviates: `tf_lookup_timeout`
  defaults to 0.1 s. The prohibition is about blocking inside a high-rate timer, and a plan is not
  periodic work — failing a plan because tf is one cycle behind is not useful. **The periodic
  consumers (tasks 11, 12, 15) keep a timeout of 0.** A consequence worth recording: under
  `use_sim_time` the timeout only expires while the simulator's clock is running.
- §6.3 calls the successful outcome "reached". In a planner it means a path was produced.
- §6.3 does not say that the pre-checks must track `eltanin`'s `plan()`. They must, in order, radius
  and model; see the duplication note above.

## `eltanin_bringup`

The package holds no code: launch, parameters, RViz configuration and the tests that fence them. One
launch file exists so far, covering the global half of the stack — a map, `global_costmap` and
`global_path_planner`. Nothing here drives a robot: there is no controller and no `cmd_vel`
publisher in the stack yet, so this cannot move anything.

### Running it, from a clean shell

Five terminals, all of them with the same environment. Steps 1 and 2 are once per machine; 3 to 6
are the run.

**1. Environment.** Every terminal below needs these three lines. `rmw_zenoh_cpp` is not in
`/opt/ros/jazzy` on the development machine, hence the second one (see *Requirements*).

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

**2. Build, then source the workspace.**

```bash
cd ~/workspace/eltanin_ws
colcon build --symlink-install --packages-ignore eltanin \
  --cmake-args --no-warn-unused-cli -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

**3. The zenoh router** — `rmw_zenoh_cpp` discovers through it, so it comes up before any node.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**4. Two transforms, by hand.** There is no localization and no simulator yet, and **RViz draws
nothing at all without a transform** — see *There is no localization yet* below for why, and why
this is not in the launch file. Two rather than one, so that a goal given in `odom` exercises the
planner's tf path. Replace the zeros with a pose in free space if the map's origin is not free.

```bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_footprint
```

**5. The stack.** `map` names the static map to load; leave it out and no `map_server` starts, which
is the other ingestion path.

```bash
ros2 launch eltanin_bringup eltanin_bringup.launch.py \
  map:=$(ros2 pkg prefix navyu_navigation)/share/navyu_navigation/map/map.yaml
```

Wait for these two lines before giving a goal — the second one is the costmap the planner needs:

```
[map_server] Read map ...: 4000 X 4000 map @ 0.05 m/cell
[global_costmap] built a 4000x4000 costmap at 0.050000 m from /map, 0 observation cells
```

**6. A goal.** RViz has no goal tool on purpose (nothing subscribes to `/goal_pose` until task 13),
so goals are given on the action. `use_start: false` reads the start from tf; pass `use_start: true`
with a `start` pose to plan from somewhere else.

```bash
ros2 action send_goal /global_path_planner/compute_path_to_pose \
  eltanin_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map},
           pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}},
    use_start: false}"
```

The result carries `outcome`, `message` and the path. `outcome: 0` is success; every other value is
explained in the `eltanin_planner` table above, and `message` is the same single line the node logs.

**What to look at.** In RViz: `StaticMap` and `GlobalCostmap` (the second one shows the inflation),
`GlobalPath` once a goal succeeds, and `FootprintPath` for the robot outline along it. On the
command line:

```bash
ros2 topic echo --once /global_costmap/global_costmap --field header   # the belief exists
ros2 param get /global_costmap robot.footprint                        # and both nodes agree
ros2 param get /global_path_planner robot.footprint                   # on the machine
```

**Rebuilding the costmap.** There is no timer. If the first plan comes back `INPUT_STALE`, or after
`clear_observations`, republish the whole area:

```bash
ros2 service call /global_costmap/update std_srvs/srv/Trigger
```

**Shutting down.** `Ctrl-C` in each terminal, the launch one last if the order matters to you.
Nothing persists between runs.

### Arguments

| Argument | Default | Meaning |
|---|---|---|
| `robot_profile` | `kachaka` | Which `config/robot/<name>.yaml` the whole stack reads. `sim_robot` is the other one. |
| `use_sim_time` | `false` | Follow `/clock`. Pass `true` with the simulator; the planner's tf timeout only expires while that clock runs. |
| `use_composition` | `true` | One container, or one process per node. Both paths run the same registered component; the separate one sends 16 MB across a process boundary on every update and is for debugging. |
| `use_rviz` | `true` | Start RViz with `rviz/eltanin.rviz`. |
| `map` | empty | Map yaml for `nav2_map_server`. **Empty means no `map_server` at all**, and the static map then comes from whoever else publishes `/map`. |
| `params_file` | `config/navigation.yaml` | Node-specific parameters, keyed by node name. |
| `rviz_config` | `rviz/eltanin.rviz` | RViz configuration. |

Every argument is declared, every declared argument is read, and `test_launch_file.py` fails if
either stops being true. navyu's `localization.launch.py` read one it never declared, so passing it
changed nothing and `--show-args` did not list it.

**Parameters are written once.** `config/robot/*.yaml` uses the `/**` wildcard, so `robot.*` and
`frames.*` reach every node as the same number — which is what keeps `global_costmap`'s inflation
threshold and `global_path_planner`'s `Free` boundary in agreement. `config/navigation.yaml` holds
only what is specific to one node, keyed by node name. No key appears in both files. navyu had four
copies of the same values and they had drifted apart.

### The static map has two ways in, and `map` picks between them

`global_costmap` subscribes to `/map` and nothing else; it never opens a file. The two ingestion
paths are therefore two ways of getting something onto that topic:

| `map` | What happens |
|---|---|
| a yaml path | `nav2_map_server` reads the file and publishes it `transient_local`. |
| empty (default) | No `map_server` is started, and `/map` is expected from elsewhere — the kachaka bridge's `/kachaka/mapping/map`, a SLAM node, or a `map_server` run by hand. |

**The default is empty on purpose.** Pointing it at `navyu_navigation`'s map would be convenient and
wrong twice: this stack replaces navyu, so depending on it at runtime is backwards, and
`DeclareLaunchArgument` resolves its default value whether or not anything reads it — a
`FindPackageShare("navyu_navigation")` default aborts the whole launch in a workspace without navyu,
even with the map server switched off. Task 22 gives the package a map of its own.

**Do not republish the same map as a keep-alive.** A second `/map` message replaces the costmap
object and drops every accumulated observation with it, for the reason in *Why accumulating state
does not break the origin rule* above.

### Composition is what makes the 16 MB affordable

Both components are loaded with `use_intra_process_comms`, so the whole-area costmap is shared as a
pointer rather than serialized on every update. That does not cost the planner its late subscription:
`rclcpp`'s intra-process manager replays a `transient_local` publisher's last message to a
`transient_local` subscription, and both nodes take the message as a `ConstSharedPtr` and copy only
what they keep.

`nav2_lifecycle_manager` is not part of this install, so the two `map_server` transitions are driven
by `ros2 run nav2_util lifecycle_bringup` instead of a manager node.

Two things this launch file does not have yet: `simulation.launch.py` and `kachaka.launch.py` (tasks
10 and 22), and the `autostart_output` argument, which belongs to `collision_predictor` (task 12) and
is not declared while there is nothing to switch.

### There is no localization yet, so tf comes from the command line

Nothing in this launch file publishes a transform, and **RViz draws nothing at all without one** —
`tf2` answers `canTransform` for two frames it has never heard of with false, even when they have the
same name, so a map whose `header.frame_id` is `map` does not appear under a fixed frame of `map`
until `map` exists in the tree. `use_start: false` needs the same transform for a different reason:
that is where the planner reads the robot's pose.

Until localization (task 22) or the simulator (task 10) owns it, publish it by hand — step 4 of the
procedure above.

This deliberately stays out of the launch file. An argument that fabricates the robot's pose is one
`kachaka.launch.py` could inherit, and a fixed transform on the real robot is a plan computed from
the wrong place.

### What M2 asked for, and how to see it

| Condition | How to check |
|---|---|
| The static map is ingested both ways | Launch with `map:=<yaml>`; then launch with `map:=''` and run `map_server` by hand. `/global_costmap/global_costmap_visual` appears either way. |
| The global costmap is built and published | `ros2 topic echo --once /global_costmap/global_costmap --field header`. There is no rate to measure: the node has no timer. |
| A goal produces a path | The action result carries `outcome: 0` and the path, and `/global_path_planner/global_path` carries the same one. The last pose keeps the yaw that was asked for. |
| RViz shows map, costmap and path | The two `Map` displays and `GlobalPath`, with the transforms from step 4. The costmap uses the `costmap` colour scheme, without which the inflation is one flat shade. |
| An unreachable goal fails visibly | A goal inside a wall returns `outcome: 5` (`OUTCOME_NO_PATH`) with a `message`; a goal off the map returns `outcome: 2` (`OUTCOME_START_GOAL_FAILED`). Neither is a silent empty path, which is what navyu produced. |
| A frame mismatch is detected | A goal in `odom` is transformed and succeeds. A goal in a frame tf has never seen returns `outcome: 2`, and the log line names both frames, the stamp and the timeout. An empty `frame_id` is refused rather than assumed to be `map`. |

Run on navyu's 4000x4000 map with the transforms of step 4, `use_rviz:=false`, and — **unlike the
procedure above** — the default `rmw_fastrtps_cpp` rather than `rmw_zenoh_cpp`, so the numbers below
say nothing about zenoh. `map_server` read the map in 0.24 s and `global_costmap` built the whole
area 0.28 s later; `lifecycle_bringup` drove both transitions without a manager node.

| Goal | Result |
|---|---|
| `(5.0, 3.0)` in `map`, `use_start: true` from `(0, 0)` | `outcome: 0`, 212 poses, 11.43 m, searched in 68.5 ms |
| the same with `use_start: false` | identical, so the pose read from tf matches the one passed by hand |
| `(90, 90)`, unknown space | `outcome: 5` — *rejected goal cell (3799, 3799): cost 255 classifies as Inscribed, not Free; the goal is reported, not moved* |
| `(500, 0)`, off the map | `outcome: 2` — *it lies outside the 4000x4000 map at 0.050000 m from (-100.000000, -100.000000)* |
| `(5.0, 3.0)` in `odom` | `outcome: 0` — transformed, then planned |
| the same in `nowhere` | `outcome: 2` — *'nowhere' to frames.map 'map' at 0 ns is not available within 0.100000 s* |
| the same with an empty `frame_id` | `outcome: 2` — *its header.frame_id is empty; frames.map 'map' is not assumed* |

`ros2 param get` on both nodes returns the same `robot.inflation_radius` and the same
`robot.footprint`, which is the property the whole parameter layout exists to guarantee.

One thing to know when checking by hand: `ros2 topic echo --once
/global_path_planner/global_path` after the plan has finished prints nothing, because that topic is
`volatile` on purpose (a late subscriber must not be handed an abandoned goal's path). Echo it in
another terminal before sending the goal, or read the path out of the action result.

### The fences

`colcon test` runs four pytest files over the configuration itself. They exist because none of what
they check fails at startup:

| Test | What it refuses |
|---|---|
| `test_robot_profile_config.py` | A profile missing a key (it would fall back to the C++ default and stop describing the machine), a footprint written with integer literals, and a kachaka profile that has drifted from `declare_robot_profile()`'s defaults or stopped being the collision box. |
| `test_navigation_config.py` | A node-name key no node answers to, a machine value written here as well as in `robot/*.yaml`, a number whose YAML spelling gives it the wrong parameter type, and smoother weights that diverge. |
| `test_rviz_config.py` | A display whose topic nothing publishes, a display whose publisher is switched off in `navigation.yaml`, the wrong colour scheme on the costmap, and a tool that publishes a topic nobody reads. |
| `test_launch_file.py` | An argument read but not declared, an argument declared but not read, the single-threaded container, a component without `use_intra_process_comms`, and the two start-up paths passing different parameters. |

The `test_rviz_config.py` gate is the one worth knowing about when adding a display: a topic that
only exists when a parameter is true has to have that parameter turned on in the same commit. That is
why `publish_footprint_path` is `true` in `navigation.yaml` — `FootprintPath` is in the RViz
configuration, and a display of a topic that is never published is the navyu defect this fence names.

RViz's Map display logs one shader link error (`indexed_8bit_image.vert`, "active samplers with a
different type refer to the same texture image unit") on some drivers. It is an upstream rviz2
issue, not a problem with the configuration; the display is still created.

### The first plan after startup can come back INPUT_STALE

Observed once under `rmw_zenoh_cpp` on the 4000x4000 map: `map_server` was `active`,
`global_costmap` had built its costmap, and `/global_costmap/global_costmap` had one publisher and
one subscriber — yet `global_path_planner` answered `OUTCOME_INPUT_STALE`, meaning the whole-area
message it is `transient_local` on had not reached it. One call to

```bash
ros2 service call /global_costmap/update std_srvs/srv/Trigger
```

republishes the whole area and every plan after that succeeds.

The cause is not established: it is either the durability replay of a 16 MB sample to a
late-joining subscription, or a discovery race between the two components in the same container.
It is intermittent — the same launch has also come up planning correctly on the first goal. What is
certain is that both sides agree on `KeepLast(1)` / reliable / `transient_local`, and that the
belief exists on the publishing side while the subscriber says it has nothing. **Worth settling
before task 13 makes `navigator` depend on the first plan succeeding**; until then, `~/update` is
the workaround, and it is the call `navigator` is expected to make before planning anyway.

**This was observed before the components were given `use_intra_process_comms`**, which moves the
replay from the middleware to `rclcpp`'s intra-process manager and takes `rmw_zenoh_cpp` out of that
path entirely. Whether the symptom survives the change has not been rechecked, so neither the
observation nor its absence should be taken as settled.

### Corrections to the design document

- §6.3 and the `eltanin_planner` topic table above say the launch file remaps the costmap
  subscriptions. It does not, and it should not: both nodes keep their fixed names in the root
  namespace, where `global_costmap/global_costmap` already resolves to what `global_costmap`
  publishes as `~/global_costmap`. A remap would be a line of wiring with no reason a reader could
  find.
- §7 lists the launch arguments but not their defaults. `map` is empty, which means no `map_server`;
  see the two ingestion paths above for why the alternative aborts the launch.
- §7 does not say how `map_server` is started. It is a lifecycle node and `nav2_lifecycle_manager` is
  not part of this install, so `nav2_util`'s `lifecycle_bringup` drives the two transitions.
- §7 lists `autostart_output` among this launch file's arguments. It is not declared here: its
  consumer is `collision_predictor` (task 12), and until then it could only be an argument that does
  nothing when passed.
- §7 does not mention tf. Nothing here publishes one, and RViz needs one before it draws anything;
  the commands are above.

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
`eltanin_ros_common`; if the conversion you need is missing, add it there —
`apply_costmap_update()` is there because `global_path_planner` needed it, and it is the inverse of
`to_costmap_update_msg()`: what one cuts out the other writes back, and a patch that fails any check
leaves the map untouched rather than half applied. Cell copies in that
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
