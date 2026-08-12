# eltanin_ros

ROS 2 packages that connect [`eltanin`](https://github.com/RyuYamamoto/eltanin) — a ROS-independent
C++20 2D navigation library — to ROS 2, and replace the [`navyu`](https://github.com/RyuYamamoto/navyu)
navigation stack. All planning, control and costmap logic lives in `eltanin`; the packages here own
only the ROS boundary: parameters, topics, TF, clocks and lifecycle.

The design is recorded in [`docs/design/eltaninnavyuros.md`](docs/design/eltaninnavyuros.md),
which is written in Japanese.

## Safety notice

> **`collision_predictor` starts with its output disabled, and turning it on is the moment the
> robot can move.** `ros2 service call /collision_predictor/enable_output std_srvs/srv/SetBool
> "{data: true}"` makes the node publish `/cmd_vel` from the next cycle; if `cmd_vel_raw` and
> `local_map` are both fresh at that moment, **the robot drives immediately, dock or no dock**
> (design §7.1 / F-17 / R-5). `output_enabled_on_startup` is `false` in the shipped configuration
> for exactly this reason, and the confirmation that the robot is off its dock is a human step that
> must not be automated.
>
> **TODO (task 23).** The rest of the real-robot (kachaka) startup sequence is written here once it
> has been validated on hardware. Until then, treat every launch file in this repository as
> simulation-only.

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
| CMake | 3.20 or later; 3.24 for `eltanin_vendor` |
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

[docs/build.md](docs/build.md) covers two rebuild traps that produce confusing failures, and the
`eltanin_vendor` cache variables.

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
| `eltanin_controller` | `path_follower` and `collision_predictor` nodes | **both implemented** |
| `eltanin_navigator` | orchestrator | not implemented (tasks 13, 21) |
| `eltanin_simulator` | `simple_simulator` node | not implemented (task 10) |
| `eltanin_bringup` | launch / config / rviz / map, plus the `goal_pose_relay` script | **`eltanin_bringup.launch.py` implemented**; simulation and kachaka bringup in tasks 10 and 22 |

The metapackage's `package.xml` lists one `exec_depend` per implemented package, so the list is also
the list of what exists.

## Documentation

Per-package usage guides in English live under [`docs/`](docs/). The design document
`docs/design/eltaninnavyuros.md` is in Japanese and records the decisions taken before
implementation; where the code diverged from it, the guides carry the correction.

## License

Apache-2.0, matching `eltanin` and `navyu`. A `LICENSE` file is not yet committed to this
repository; source files carry the Apache-2.0 header.
