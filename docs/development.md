# Development

Conventions for working in this repository: hooks, style, lint, the API rules the nodes follow,
and how parameters and tests are expected to behave.

The design reasoning is in the Japanese design document [`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md).

## pre-commit

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

## Style and lint

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

## Compiler warnings

Each package adds `-Wall -Wextra -Wpedantic` with
`target_compile_options(<target> PRIVATE ...)` and promotes them to errors when
`ELTANIN_ROS_ENABLE_WERROR` is `ON`. CI passes `-DELTANIN_ROS_ENABLE_WERROR=ON`. These flags are
never set through `CMAKE_CXX_FLAGS`, because that would leak them into the `eltanin_vendor`
ExternalProject and into `rosidl` generated code.

## API rules

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

## Parameters have no defaults in code, with one deliberate exception

**Every node refuses to start on a key that is not set.** There are no fallbacks in
`declare_parameter`, and `declare_robot_profile()` has none either. A missing key is one `ERROR`
line naming it and a `std::runtime_error` from the constructor.

**`collision_predictor` is the exception**, and it is one on purpose: it declares all thirteen of
its own keys with defaults so that it starts on a machine profile alone. It owns `/cmd_vel`, so
"cannot start" is the worst outcome available to it, and the parameter whose wrong value would be
dangerous — `output_enabled_on_startup` — defaults to the safe side. The silent-typo cost of a
default is paid back by `test_watchdog.cpp`, which fails when the shipped yaml and the declared key
set disagree in either direction. The machine profile is still mandatory for it, so nothing about
`robot.*` or `frames.*` changes.

This reverses design §7's F-14, which asked for a meaningful default behind every parameter, on the
grounds that navyu crashed at startup on a key mismatch. **That reasoning is backwards.** Crashing
on a key mismatch is the behaviour worth having: it is loud, immediate, and names the key. navyu's
actual defect (N-6) was the same footprint declared in four nodes under keys that had drifted apart
— fixed by making `declare_robot_profile()` the single declaration point, not by adding fallbacks.
What the fallbacks added was a silent failure: a typo in a yaml key means the node runs on a code
default and nobody notices. The worst case was the machine profile, whose defaults were kachaka's
measurements, so any other robot's profile with a missing `robot.footprint` would have driven with
kachaka's collision box.

Consequently:

| | |
|---|---|
| Every node ships a complete config | `eltanin_costmap/config/global_costmap.param.yaml`, `eltanin_planner/config/global_path_planner.param.yaml`, `eltanin_controller/config/path_follower.param.yaml`. The package that declares a key ships the file that sets it. |
| The machine profile lives in `eltanin_ros_common` | `config/robot/<name>.yaml`. Every node package depends on it, so every node package's tests can read it without a dependency cycle. |
| Each node's ROS test starts the node from those shipped files | That is what proves the file is complete; a key added to a node and not to its yaml fails in CI. |
| `eltanin_bringup` composes, it does not own | `costmap_params_file` and `planner_params_file` point at the packages. There is no `navigation.yaml`. |

**A launch argument cannot override a node-named key.** rcl resolves a parameter by how specific
the node name is, not by the order the sources were given: a key under `path_follower:` beats the
same key under `/**` however late `/**` arrives. An inline parameter dictionary in a launch file and
a command-line `-p name:=value` **both land under `/**`**, so neither can override a shipped config.
An argument that has to win must be written to a file under the node's own name;
`eltanin_kachaka_demo/launch/kachaka_follower.launch.py` does that in an `OpaqueFunction`.

## Tests that start nodes need their own ROS domain

`colcon test` shares the ROS graph with whatever else is running. A live stack on the same machine
makes `count_publishers()` see its publishers, and a `rmw_zenohd` router **retains `/tf_static`**,
so a test that expects a transform lookup to fail sees it succeed. Run them isolated, and one
package at a time where two packages publish the same topic names:

```bash
ROS_DOMAIN_ID=77 colcon test --executor sequential --packages-ignore eltanin
```

Frame names and node names should be private to the test as well
(`eltanin_controller/test/test_path_follower.cpp` remaps both), but that alone is not enough: the
domain is what actually separates them.

## Branches

Work happens on `dev`. `main` is updated only through a pull request from `dev` (D-28); never push
to `main` directly.
