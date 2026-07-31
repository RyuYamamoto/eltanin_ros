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
| `eltanin_ros_common` | conversions, clock, watchdog, TF, parameter validation | **conversion layer implemented**; clock, watchdog and robot profile in task 5 |
| `eltanin_costmap` | `global_costmap` and `local_map` nodes | not implemented (tasks 6, 15) |
| `eltanin_planner` | `global_path_planner` and `local_path_planner` nodes | not implemented (tasks 7, 17, 19) |
| `eltanin_controller` | `path_follower` and `collision_predictor` nodes | not implemented (tasks 11, 12) |
| `eltanin_navigator` | orchestrator | not implemented (tasks 13, 21) |
| `eltanin_simulator` | `simple_simulator` node | not implemented (task 10) |
| `eltanin_bringup` | launch / config / rviz / map, no code | not implemented (tasks 8, 22) |

The metapackage's `package.xml` lists one `exec_depend` per implemented package, so the list is also
the list of what exists.

## The conversion layer (`eltanin_ros_common`)

Everything that crosses between `eltanin`'s types and ROS 2 messages is converted here and nowhere
else: cost value ranges, quaternions, twists, transforms, scans, maps and paths. The package has no
nodes and does not depend on `rclcpp`, so all of its behaviour is checked by `colcon test` without
starting a node.

Conversions never throw. A conversion that can fail returns `ConversionResult<T>`, a check that
produces no value returns `ConversionStatus`, and a rejection carries **one line** naming the frame,
the resolution, the size, the field that was wrong, its value and the tolerance. Failure is not
logged here — `eltanin_ros_common` has no logger; the caller logs it. For conditions that repeat
every cycle, `WarnOnceLatch::should_warn()` answers whether this is the first occurrence.

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

### Types whose first real user comes later

`WarnOnceLatch` and `StampedScan` have no node using them yet — the tests are their only callers.
They exist because the acceptance conditions they serve cannot be met otherwise: a node must not be
able to invent a `laser_frame`, so the scan conversion returns the frame and stamp together with the
`ScanData` (which carries neither), and a broken 2D assumption must not be silently dropped nor
logged once per cycle. First users are tasks 11, 12 and 15.

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
