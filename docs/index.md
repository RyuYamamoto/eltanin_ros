# eltanin_ros

ROS 2 packages that connect [`eltanin`](https://github.com/RyuYamamoto/eltanin) — a ROS-independent
C++20 2D navigation library — to ROS 2, and replace the
[`navyu`](https://github.com/RyuYamamoto/navyu) navigation stack. All planning, control and costmap
logic lives in `eltanin`; the packages here own only the ROS boundary: parameters, topics, TF, clocks
and lifecycle.

Source: [github.com/RyuYamamoto/eltanin_ros](https://github.com/RyuYamamoto/eltanin_ros)

!!! danger "Enabling `collision_predictor` output is the moment the robot can move"
    The node ships with its output disabled. Calling
    `/collision_predictor/enable_output` makes it publish `/cmd_vel` from the next cycle, and if
    `cmd_vel_raw` and `local_map` are both fresh at that moment **the robot drives immediately, dock
    or no dock**. Confirming that the robot is off its dock is a human step that must not be
    automated — see
    [The output starts disabled](controller.md#the-output-starts-disabled-and-disabling-it-means-stopping).
    Every launch file in this repository is simulation-only until the hardware startup sequence has
    been validated.

## Packages

<!-- Keep this table in sync with the one in README.md. -->

Ten packages are planned. Everything not yet written is listed with the task that creates it, so that
nothing looks available before it is.

| Package | Contents | Status |
|---|---|---|
| `eltanin_ros` | metapackage (`ament_package` only) | **implemented** |
| `eltanin_vendor` | builds and installs `eltanin` via `ExternalProject` | **implemented** |
| `eltanin_msgs` | msg / action definitions, interfaces only | **implemented** |
| `eltanin_ros_common` | conversions, clock, watchdog, TF, parameter validation | **conversions, clock, watchdog and robot profile implemented** |
| `eltanin_costmap` | `global_costmap` and `local_map` nodes | **`global_costmap` implemented**; `local_map` in task 15 |
| `eltanin_planner` | `global_path_planner` and `local_path_planner` nodes | **`global_path_planner` implemented**; `local_path_planner` in tasks 17 and 19 |
| `eltanin_controller` | `path_follower` and `collision_predictor` nodes | **both implemented** |
| `eltanin_navigator` | orchestrator | not implemented (tasks 13, 21) |
| `eltanin_simulator` | `simple_simulator` node | not implemented (task 10) |
| `eltanin_bringup` | launch / config / rviz / map, plus the `goal_pose_relay` script | **`eltanin_bringup.launch.py` implemented** |

## Getting started

The workspace layout, requirements and the build command are in the
[README](https://github.com/RyuYamamoto/eltanin_ros#build). Once it builds, follow
[eltanin_bringup](bringup.md) to launch the global half of the stack.

## Guides

Written in English, one per package.

- [Build pitfalls](build.md) — two rebuild traps, and the `eltanin_vendor` cache variables
- [eltanin_ros_common](ros-common.md) — the conversion layer, what it does and does not preserve, and
  the runtime pieces
- [eltanin_costmap](costmap.md) — `global_costmap`: why it has no timer, and how accumulated
  observations stay consistent with the map origin
- [eltanin_planner](planner.md) — `global_path_planner`: the action, its six outcomes, and how the
  four failure causes are told apart
- [eltanin_controller](controller.md) — `path_follower` and `collision_predictor`, and why the split
  matters
- [eltanin_bringup](bringup.md) — launching it from a clean shell, arguments, and the fences
- [Development](development.md) — hooks, style, lint, API rules, parameters and tests

## Design document

`docs/design/eltaninnavyuros.md` records the decisions taken before implementation, keyed by the
requirement identifiers (C-n / N-n / E-n / F-n / D-n / Q-n / R-n / M-n / NF-n). **It is written in
Japanese and is not part of this site**; it lives in the repository and is read on
[GitHub](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md). It
will be added here once translated.

The design document is never edited to match the code. Where the implementation diverged, the
difference is recorded under *Corrections to the design document* in the guide for that package.

## License

Apache-2.0, matching `eltanin` and `navyu`. Source files carry the Apache-2.0 header.
