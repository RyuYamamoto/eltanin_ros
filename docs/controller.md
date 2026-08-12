# `eltanin_controller`

Two nodes. `path_follower` turns a path into a **requested** velocity on `~/cmd_vel_raw`;
`collision_predictor` is the only consumer of that topic and the single owner of `/cmd_vel`. The
split is the whole point: the follower may be restarted, reconfigured or blocked without `/cmd_vel`
ever going quiet, and the node that owns the wheels runs outside every action.

The design reasoning is in the Japanese design document
[`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md),
referenced by section number. Where the implementation diverged from it, the difference is
recorded under *Corrections to the design document* below rather than by editing the design
document.

## `path_follower`

### The publish never stops, and that is a property of the code

navyu returned from its control callback without publishing anything when it had no path or could
not find the robot, which leaves the last command sitting on the driver. Here every failure is a
value returned from `run_cycle()`, and `on_timer()` is two lines: run the cycle, publish the cycle.
There is no early `return` in `on_timer()` to forget a publish in, so "a zero command still goes out"
is not a rule a reviewer has to check. `~/cmd_vel_raw` and `~/diagnostics` are published on every
cycle without exception; `~/lookahead_point` only while tracking, because the point is zero
otherwise and drawing it at the origin would be a lie.

### The composition, and why `apply_linear_limit()` is not `std::min`

`GoalApproach` runs first every cycle. Its state decides whether `PurePursuit` runs at all:

| `approach.state` | what happens | command |
|---|---|---|
| `Reached` | terminal, latched | zero |
| `AlignmentTimeout` | terminal, latched | zero |
| `Aligning` | **`PurePursuit` is not called** | `approach.command` |
| `Inactive` / `Approaching` | `PurePursuit` runs | `apply_linear_limit(tracking, approach.linear_vel_limit)` |

Not calling `PurePursuit` while aligning follows `eltanin`'s own `examples/navigation_loop.hpp`,
which is the only primary source for the order; the pseudocode in `control-design.md` §12.1 reads as
if the final turn were also zeroed, and a robot that never turns at the goal is the result.

The limit is applied with `eltanin::control::detail::apply_linear_limit()`, which scales the whole
twist so `w / v` is preserved. Capping `linear.x` alone with `std::min` leaves the angular velocity
untouched and inflates the curvature exactly where the robot is slowing into the goal, which is the
oscillation the approach exists to remove. `test_command_composition.cpp` pins the numbers:
`v = 0.5, w = 0.4` under a limit of `0.25` becomes `(0.25, 0.2)`, and the `std::min` version's
`(0.25, 0.4)` fails the test.

`compose()` is the only place either result is read, which is what kept the move to `eltanin`'s
`PathFollower` interface — `compute()` becoming `follow()`, `PurePursuit::Result` becoming
`FollowResult` — down to that one function, its signature and its tests.

### `ok = false` means three things, and the navigator can treat it as one

`~/diagnostics` carries `ok` among its values. It is false only where the follower cannot recover on
its own and
would sit at zero forever while the upstream keeps sending the same input:

- `GoalApproach` reported `AlignmentTimeout` — it latches until `~/reset`
- the follower reported `NoPath`
- the follower reported `GoalReached` before `GoalApproach` accepted the goal
- the follower reported `SolverFailed`, which pure pursuit never does and a solving follower will

Everything on the input side — no path yet, stale, empty, wrong frame, no transform, no usable
elapsed time — leaves `ok` true and is reported through `reason` and `message` instead. Those clear
themselves the moment the upstream comes back, and the navigator watches `local_map` and `/cmd_vel`
for itself. So the navigator's rule can be exactly "`ok == false` is a stopping trigger".

`NoPath` in that list is defensive: it is returned only for an empty path, and the node rejects an
empty path before the follower is reached. A one-pose path comes back as `GoalReached`, not
`NoPath`.

### Nothing resets the follower except `~/reset`

The follower never resets itself when the path changes. Whether a replan should drop the velocity
ramp depends on **why** the replan happened, and only the navigator knows that: a replan triggered
by an observation while driving must not decelerate, one triggered by a stop must. So `~/reset`
(`std_srvs/Trigger`) is the single entrance. The service sets a flag and returns immediately;
`success` is always true and the message says the reset applies before the next command, because
claiming it already happened would be a lie. The `PeriodicClock` is deliberately not reset with the
two controllers — that would make the next cycle a first tick and add one extra zero command.

**Operational consequence while there is no navigator:** `Reached` and `AlignmentTimeout` latch, so
after the first goal the follower will not follow a second path until `~/reset` is called by hand.
`approach_state` stays at `reached` on `~/diagnostics`, so the state is visible rather than
mysterious.

```bash
ros2 service call /path_follower/reset std_srvs/srv/Trigger
```

### One input, chosen at startup, with two different meanings of "too old"

`path_source` selects which subscription is created, and only one is:

| `path_source` | topic | type | deadline |
|---|---|---|---|
| `path` (default) | `global_path_planner/global_path` | `nav_msgs/Path` | `path_timeout`, default **0 = none** |
| `trajectory` | `local_path_planner/local_trajectory` | `eltanin_msgs/Trajectory2D` | `trajectory_timeout`, default 0.5 s, must be positive |
| `directed_path` | `global_path_planner/global_path_directed` | `eltanin_msgs/DirectedPath` | `path_timeout`, as for `path` |

The two deadlines are separate parameters because the inputs are not comparable. A global path is
published once per replan, so a 0.5 s deadline would make it stale immediately and the follower
would sit at zero forever; a local trajectory arrives at the control rate, so no deadline at all
would be the navyu failure again. `path_timeout: 0` therefore means "no deadline" — not a very long
one — and `PathInput` implements that rather than `StaleInput`, whose rule that a non-positive
timeout is always stale exists on purpose and is left alone. Where there is a deadline it is checked
on both sides: a stamp from the future is stale too.

Both topic names are relative and can be remapped. A path whose `header.frame_id` is not `frames.map`
is rejected rather than transformed: transforming periodic input would turn a tf failure into a third
state that is neither "no path" nor "an old path". A rejection does not throw away a path already
being followed — it only declines the new one.

The velocity annotation on `Trajectory2D` (`linear_velocity`, `angular_velocity`, `time_from_start`)
is dropped by `eltanin_ros_common::to_path()`, because `eltanin` has no trajectory type and
`PurePursuit` cannot consume a velocity profile yet.

### What the defaults actually do

The node declares no `max_angular_vel` of its own: `robot.max_angular_vel` is fed into both
generators, as `eltanin`'s `control-design.md` §12.2 asks. `desired_linear_vel` is a cruise speed
rather than a body limit, so it is declared — but it is capped at `robot.max_linear_vel` with a
`WARN` rather than refused, or the default profile could not start. With the shipped defaults that
cap fires:

| | default | effective | from |
|---|---|---|---|
| `desired_linear_vel` | 0.5 | **0.30** | capped at `robot.max_linear_vel`, one `WARN` line |
| `max_angular_vel` | not declared | **1.57** | `robot.max_angular_vel` |
| in-place turn rate | — | **0.785 rad/s** | `PurePursuit` turns at half of `max_angular_vel` |
| `dt` bound for the in-place turn | — | **0.089 s** | `yaw_tolerance / (0.5 * max_angular_vel)` |

That last row is worth watching. At 20 Hz the cycle is 0.05 s, so the margin is 1.78x, not the 2.8x
the design assumed with `eltanin`'s own default of 1.0 rad/s. Above the bound the bang-bang in-place
turn overshoots the tolerance band every cycle and the robot spins without setting off. It applies
only to `PurePursuit`'s two in-place turns — the initial alignment and the re-alignment near the
last pose. `GoalApproach`'s final turn is proportional and converges for any `dt` below 0.5 s.

`~/diagnostics` publishes `control_dt` every cycle so the real distribution can be read with
`ros2 topic echo` instead of guessed. If it does turn out to be a problem, lower
`robot.max_angular_vel` or raise `yaw_tolerance`; raising `update_frequency` is the last resort,
since more cycles is also more jitter.

### The two `eltanin` calls are not wrapped in a `try`

`GoalApproach::compute()` and `PathFollower::follow()` both throw `std::invalid_argument` on a
non-finite pose or a non-positive `dt`. Neither can arrive: `to_transform2d()` validates the pose
before it becomes one, and
`tick.usable()` is the only gate `dt` passes through. Catching would hide the day one of those two
stops holding. If the process does die, the system still fails safe — `~/cmd_vel_raw` stops, and
`collision_predictor`'s `cmd_timeout` zeroes `/cmd_vel`.

### Parameters and startup

`update_frequency` 20.0 / `path_source` `path` / `desired_linear_vel` 0.5 / `yaw_tolerance` 0.07 /
`lookahead_time` 0.1 / `min_lookahead_dist` 0.3 / `xy_goal_tolerance` 0.10 /
`yaw_goal_tolerance` 0.10 / `approach_distance` 0.5 / `approach_decel` 0.5 /
`yaw_align_timeout` 5.0 / `trajectory_timeout` 0.5 / `path_timeout` 0.0.

There is no `tf_lookup_timeout`: a periodic consumer keeps a timeout of 0, and a parameter would let
that be configured away. There is no `follower_type` either, because there is one follower.

A value the node cannot use is one `ERROR` line and a `std::runtime_error` from the constructor —
a load failure under `ros2 component load`, an abort under `ros2 run`. `validate()` repeats the
conditions `PurePursuit::create()` and `GoalApproach::create()` check, in their order, on purpose:
`create()` returns a bare `std::optional` and cannot say which value it disliked, so the line that
names the parameter can only be written here.

### Following a path by hand

Neither node is in `eltanin_bringup.launch.py` yet; both go in with the navigator in task 13.

```bash
ros2 launch eltanin_bringup eltanin_bringup.launch.py map:=<path-to-map.yaml>
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_footprint

ros2 run eltanin_controller path_follower
ros2 topic echo /path_follower/cmd_vel_raw
ros2 topic echo /path_follower/diagnostics
```

Give a goal with RViz's `2D Goal Pose`. `~/global_path` is `volatile` and published once per goal,
so a `path_follower` started after the planner has to be given the goal again.

### Corrections to the design document

- §6.5, D-17, §8.2 and §10 S4 all assume `PurePursuit::Status` gains an `Approaching` value. It never
  did; approach is `GoalApproach::State`'s vocabulary alone (`eltanin`'s `control-design.md` §12.6).
- §6.5 names "`control::PurePursuit` (plus the velocity-consuming overload from E-3)". The overload
  does not exist yet, so **the magnitude of `Trajectory2D`'s velocity annotation is dropped**. Its
  *sign* is not: `to_path(Trajectory2D)` turns it into the segment direction of the `eltanin::Path`
  it builds, which is the only part of it any follower can act on.
- §6.5 says a cycle with `dt <= 0` is skipped. It is skipped as far as `eltanin` goes, but
  `~/cmd_vel_raw` and `~/diagnostics` are still published: no path in this node stops the publish.
- §6.5 left `~/follower_state` as "reuse part of `NavigationState`, or put it in diagnostics". It
  first became `~/follower_state` carrying a purpose-built `eltanin_msgs/FollowerDiagnostic`; **task
  12 replaced both with `~/diagnostics` carrying `diagnostic_msgs/DiagnosticArray`**, and deleted
  the message. No information was lost — `status`, `approach_state`, `reason`, `ok` and the numbers
  are all `values` entries now — and two things went away with the wire format: the 35-line
  `to_wire()` tables that made an enum's declaration order a wire format, and `eltanin_msgs` as a
  dependency of `command_composition`. `reason` is an `enum class FollowerReason` and reaches the
  wire as a name. See the `collision_predictor` section for the vocabulary the two nodes share.
- §6.5 lists `max_angular_vel` among the node's parameters. It is not declared here;
  `robot.max_angular_vel` reaches both generators.
- `PurePursuit::Status::NoPath` is returned only for an empty path, which the node rejects before
  `compute()`. A one-pose path returns `GoalReached`. `NoPath` is reachable in unit tests only.
- §6.5's `dt` bound is stated for `max_angular_vel` 1.0. With `robot.max_angular_vel`'s default of
  1.57 it is **0.089 s**, not 0.14 s, and it binds only `PurePursuit`'s bang-bang in-place turns;
  `GoalApproach`'s final turn is proportional and unaffected.
- **The MPC is what the kachaka demo now starts by default**, which takes one build option:
  `colcon build --packages-select eltanin_vendor --cmake-args -DELTANIN_VENDOR_ENABLE_MPC=ON`,
  followed by a rebuild of everything downstream because `ELTANIN_WITH_MPC` is a `PUBLIC` compile
  definition. It is not the vendor default because it fetches OSQP and a plain build stays offline.
  Without it `path_follower` refuses to start and names the flag.
- **The MPC does not know what the robot actually executed.** It is handed
  `FollowerState{pose, nullopt}`, so `twist_of()` falls back to **the command it returned last
  cycle** — the one before `collision_predictor` limited it. Measured with a wall 0.21 m ahead: the
  MPC asks 0.15 m/s every cycle while `/cmd_vel` carries 0.061, and the MPC keeps planning from
  0.15. Its acceleration constraints are therefore applied around a speed the body is not at
  whenever the limiter bites. Pure pursuit is geometric and barely notices; the MPC is the follower
  that wants the odom subscription, and that subscription is still the open item.
- §6.5 assumes one follower. `eltanin` now has a `PathFollower` base class, a `FollowStatus` with a
  fourth `SolverFailed` value, and an MPC behind `ELTANIN_ENABLE_MPC` (default off, and
  `eltanin_vendor` does not pass it). This node calls `follow()` and reads `FollowResult`, but builds
  **no switching abstraction**: it holds a `PurePursuit` by value and declares no `follower_type`,
  because the alternative follower is not in the vendored build. `follower_type`, a
  `std::unique_ptr<PathFollower>`, a `pure_pursuit.*` / `mpc.*` parameter split and the odom
  subscription an MPC needs for `FollowerState::twist` all belong to the task that turns the MPC on.
- The paragraph above is out of date on two counts: `follower_type`, the `std::unique_ptr` and the
  `pure_pursuit.*` / `mpc.*` split all landed, and `FollowStatus` has a **fifth** value,
  `PathNotSupported`, for a path the selected follower refuses to execute. The odom subscription is
  still absent; the follower substitutes the command it returned last cycle.
- **`path_source` has a third value, `directed_path`.** `nav_msgs/Path` cannot say which way a
  segment is driven, and a follower that guesses it from the displacement and the body yaw guesses
  wrong the moment the planner changes how it samples. `eltanin_msgs/DirectedPath` carries the
  direction per segment; `global_path_planner` publishes it on `~/global_path_directed` **in
  addition to** `~/global_path`, which is unchanged, still `nav_msgs/Path`, and still what the
  action result and RViz get.
- **`mpc.min_linear_vel` may be negative.** A negative value is what allows reversing, and
  `robot.max_linear_vel` bounds it from below exactly as it bounds `mpc.max_linear_vel` from above:
  the profile declares one magnitude for both directions. 0 keeps the follower forward-only, and a
  reversing path then comes back as `STATUS_PATH_NOT_SUPPORTED` with a zero command rather than
  being mis-followed. `pure_pursuit` refuses one whatever the parameters say.
- A note on `eltanin`'s own design document rather than this one: `control-design.md` §13.3 called
  the velocity profile's floor and terminal speed `min_linear_vel` / `terminal_linear_vel`, which
  collided with `MpcFollowerParams::min_linear_vel`. One is a magnitude and the other is a signed
  bound; they are now `min_speed` / `terminal_speed`. **No ROS key changed**, because the profile
  has never been reachable from a parameter.

## `collision_predictor`

The single owner of `/cmd_vel`. It subscribes to `path_follower/cmd_vel_raw` and
`local_map/local_map`, limits the requested command against the local map with `eltanin`'s
`VelocityGovernor`, and publishes `geometry_msgs/Twist` — the one unstamped message in the stack,
because kachaka accepts nothing else (D-21).

### The watchdog is the reason this node exists

navyu had two defects, and both come from nobody owning `/cmd_vel`:

| # | navyu | symptom | here |
|---|---|---|---|
| N-1 | the control callback returned without publishing | the **last** command stays on the driver, so a lost tf keeps the robot driving | every failure is a value from `run_cycle()`; `on_timer()` is two lines and has no early `return` |
| N-2 | the limited value was written back into the subscription buffer | when the input stopped, the limit **ratcheted to zero** | the subscription writes the received value and nothing else ever writes it; the timer copies it out |

Every cycle decides in this order, and the first condition that holds is the `reason`:

| # | `reason` | `level` | `/cmd_vel` |
|---|---|---|---|
| 1 | `output_disabled` | `WARN` | **nothing**, except one zero on the disabling cycle |
| 2 | `command_rejected` | `ERROR` | zero |
| 3 | `command_missing` | `STALE` | zero |
| 4 | `command_stale` | `STALE` | zero |
| 5 | `map_rejected` | `ERROR` | zero |
| 6 | `map_missing` | `STALE` | zero — **the steady state until `local_map` exists (task 15)** |
| 7 | `map_stale` | `STALE` | zero |
| 8 | `no_transform` | `WARN` | zero |
| 9 | `outside_map` | `WARN` | zero |
| 10 | `limited` | `WARN` | the governor result |
| 11 | `none` | `OK` | the governor result |

There is no path in the code that re-sends an old command, and none that publishes nothing while the
output is enabled. `limiter_diagnostic.hpp` holds that table once; `forces_zero_command()` is what
`CycleOutcome::command` staying at its zero default means, and the two entries that carry a command
are the only ones that assign it.

### `dt` is not a gate

`path_follower` treats an unusable elapsed time as a failed cycle. This node does not: `dt` is
handed to the hysteresis and nothing else, and the hysteresis reads a non-positive step as "hold the
current, stricter limit". A cycle with no measurable elapsed time still checks the map and still
publishes.

### The output starts disabled, and disabling it means stopping

`output_enabled_on_startup` is `false`. `~/enable_output` (`std_srvs/SetBool`) is the only way to
turn the output on, and switching it back off publishes **one** zero command before going quiet —
relying on the driver's own watchdog to stop the robot would be N-1 again, one layer down.

A fourth topic, `~/predicted_footprints` (`visualization_msgs/MarkerArray`), draws the footprint at
every predicted pose in RViz, coloured by how close the body gets to an obstacle at that pose:
green from `slow_down_clearance` upward, red where it touches. A colliding rollout stops at its
last pose, and a sphere is drawn there. The clearance here is measured **under the whole
footprint**, not from a circle around it — the front reaches 0.237 m ahead of centre while the
ramp's inscribed circle is 0.12 m, so a wall right under the bumper has to read as red even where
the lateral ramp, which is about the room to the sides, is doing nothing. The two use different
measures on purpose. The `~/predicted_poses` line shows the same rollout as a path; the markers show
where along it the body gets close and where it would touch, which the line alone cannot.

The three other topics — `~/predicted_poses`, `~/footprint` and `~/diagnostics` — plus
`~/predicted_footprints` are published on **every** cycle, disabled ones included, and a disabled cycle
is a fully checked one: the transform
is looked up, the map is converted, the governor runs and only the publish is suppressed. That is
what makes enabling the output a decision rather than an experiment — `transform_ok`, `map_age`,
`clearance` and `~/predicted_poses` in RViz all answer "will it work" while the robot cannot move.
A node that is silent because it was told to be must not look like a node that died, and it must
not hide whether it is healthy either.

### The limiter law changed with this node

Driving `eltanin`'s limiter for real turned up five problems, and `eltanin` was changed rather than
the node; the reasoning lives in `eltanin/docs/collision-design.md` §2.5.1 and §3.5 to §3.10, and
only the consequences are here.

| what was wrong | what it is now |
|---|---|
| the output was effectively three-valued: with kachaka's defaults there was one intermediate step between full speed and a stop | the colliding step is bisected four times, so `collision_distance` is continuous to 1.7 mm |
| the 2 s rollout tip wandered 0.30 m under the angular jitter of pure pursuit — 2.5x the footprint half width | the horizon is derived: `reaction_time + \|v\| / robot.max_decel`, 0.9 s on kachaka. `prediction_time` is gone |
| latency was nowhere in the law | a second cap, `(collision_distance - margin) / reaction_time`, and `time_to_collision` in the diagnostic |
| a single mistaken cell was a full stop | a proximity ramp on the clearance, floored at `min_proximity_scale` so it slows down but never stops |
| the ramp measured clearance from the circumscribed circle, so a corridor the body fits through read as no room at all | it measures from the inscribed circle: the room beside the body |
| the limit snapped back the moment a cell cleared | `VelocityGovernor` releases over `release_time`; dropping stays immediate |

The enabling change is that this node does **not** hand the local map to the limiter. `local_map`
carries no inflation (D-14), which is what made the two-stage collision check unsound there (R-10).
Each received map goes through `build_distance_map()` once, and the limiter runs on the distance
map: the `Free` short-circuit becomes exactly true, and the cell value is the clearance the ramp
needs. **One missing inflation was the common cause of both R-10 and the staircase, and building a
distance field in the consumer was the answer to both.** The node keeps no costmap; the distance
map alone still identifies the obstacles, because a cell at distance 0 is one.

The ramp measures **the room beside the body**: the distance to the nearest obstacle less the
inscribed radius. The circumscribed circle was the first choice, on the grounds that it is
conservative and impossible to get wrong, and on the robot it did not survive first contact.
kachaka's circumscribed radius is 0.266 m against a half width of 0.120 m, so a corridor narrower
than 0.53 m reads as no clearance at all however the thresholds are set — while the body is 0.24 m
wide and drives a 0.5 m corridor without trouble. Measured over 697 cycles of one such corridor,
`proximity_scale` sat at its floor for the whole drive and `/cmd_vel` carried a quarter of what was
asked for. The inscribed circle is optimistic for an obstacle straight ahead, and that is the right
trade: the rollout covers ahead exactly, and what the ramp is there for is the wall alongside.

`exact_footprint_check` stays `true` by default. On a distance map it is no longer required for
correctness, only cheaper or not, and the default is kept so that handing a raw costmap to this code
some day does not silently degrade.

### Parameters and startup

`update_frequency` 20.0 / `cmd_timeout` 0.3 / `map_timeout` 0.5 / `output_enabled_on_startup` false
/ `prediction_steps` 10 / `reaction_time` 0.3 / `collision_margin` 0.2 / `exact_footprint_check`
true / `stop_clearance` 0.0 / `slow_down_clearance` 0.15 / `min_proximity_scale` 0.25 /
`release_time` 0.5 / `clearance_max_distance` 1.0.

The two clearance thresholds are tuned for kachaka and differ from eltanin's own defaults of 0.10
and 0.50, which suit its 0.6 m square outline. Read against the inscribed radius, 0.15 means full
speed from 0.54 m of corridor upwards.

`collision_margin` is tuned too: 0.10 against eltanin's 0.2, which came from navyu's safety limiter.
The margin is a fixed buffer laid on top of two laws that already account for stopping — at 0.3 m/s
the braking term contributes 0.09 m and the reaction term another 0.09 m — and in a 0.5 m corridor
the fixed part is what dominates. It showed up as a deadlock: at a dead end the plan put its cusp
0.170 m of travel from contact, 0.2 refused it for all 272 recorded cycles, and `run_index` never
left 0, so the follower sat on its first run waiting for a cusp the limiter would not let it reach.
At 0.10 the same approach runs at 0.23 m/s and stops 0.10 m short, with 0.18 m of law on top.

**This is under evaluation on the robot.** It is the one parameter here that trades stopping
distance for the ability to manoeuvre, and the alternative — keeping 0.2 and asking the planner to
leave that much room at a cusp — would cost the dead-end turns the robot needs in a 0.5 m corridor.

`robot.footprint`, `robot.max_decel`, `robot.max_linear_vel`, `frames.map` and `frames.base` come
from the machine profile and are **not** redeclared. Unlike `path_follower`, every key here has a
default, so the node starts on the robot profile alone; the shipped
`config/collision_predictor.param.yaml` writes all thirteen out anyway, and `test_watchdog.cpp`
fails if the two sets ever disagree.

`validate()` reports the first violated condition with the key, the value and the bound, in the
order `update_frequency`, `cmd_timeout`, `map_timeout`, `prediction_steps`, `reaction_time`,
`collision_margin`, `stop_clearance`, `slow_down_clearance`, `min_proximity_scale`, `release_time`,
`clearance_max_distance`, `robot.max_decel`. Three more conditions are warnings rather than errors:
`update_frequency` below 4 Hz (the kachaka bridge watchdog is 0.3 s), `exact_footprint_check` off,
and a prediction step long enough to skip through the footprint —
`max_linear_vel * horizon(max_linear_vel) / prediction_steps >= 2 * inscribed_radius`. That last one
is why raising the speed limit means raising `prediction_steps` too.

### Running it by hand, with no `local_map`

```bash
ros2 run eltanin_controller collision_predictor \
  --ros-args --params-file "$(ros2 pkg prefix eltanin_ros_common)/share/eltanin_ros_common/config/robot/kachaka.yaml"

ros2 topic echo /collision_predictor/diagnostics   # reason: output_disabled
ros2 topic hz /cmd_vel                             # nothing at all

ros2 service call /collision_predictor/enable_output std_srvs/srv/SetBool "{data: true}"
ros2 topic hz /cmd_vel                             # 20 Hz of zero, reason: map_missing
```

That last line is the whole node in one measurement: with no map, no command and no tf it publishes
a zero command twenty times a second rather than nothing.

### The temporary wiring it replaces

The uncommitted `eltanin_kachaka_demo` drives the robot through `cmd_vel_relay.py` and
`navyu_safety_limiter`, and both exist only because this node did not. **Removing them is task 22 /
23**, not this task: the swap has to be verified by driving the robot.

### Corrections to the design document

- **§6.6 and §7.1 name `eltanin_msgs/SetEnabled` in one place and `std_srvs/SetBool` in another.**
  It is `std_srvs/SetBool`; `eltanin_msgs` has no `srv/` directory and this service needs nothing a
  `SetBool` does not have.
- **`~/diagnostics` is `diagnostic_msgs/DiagnosticArray`, published at 20 Hz on the node-relative
  topic.** It is deliberately not sent to the aggregating `/diagnostics`, whose convention is about
  1 Hz; a launch remap or an analyzer is where aggregation belongs. `path_follower` moved to the
  same shape, and `eltanin_msgs/FollowerDiagnostic` was deleted.
- **`max_deceleration` is not a parameter of this node.** §6.6 lists it; it is `robot.max_decel`,
  because "how hard can this machine brake" must not have three different answers.
- **"disabled means publish nothing" applies to `/cmd_vel` only.** The diagnostic, the predicted
  poses and the footprint go out every cycle, and the enabled-to-disabled transition publishes one
  zero command.
- **`outside_map` is a stopping condition the design document does not have.** `limit()` truncates a
  rollout that leaves the map without limiting anything, so the command it returns was never
  checked; publishing it would be reporting a clearance nobody measured.
- **§6.7 has the navigator subscribing to `collision_predictor/cmd_vel`.** The topic is the relative
  name `cmd_vel`, so it resolves to `/cmd_vel` in the root namespace, and that is what a consumer
  subscribes to.
- **`prediction_time` no longer exists** and `reaction_time` replaced it. The horizon is derived
  from the requested speed, so it is not a number anyone sets.
- **Parameters are declared with defaults here**, unlike `path_follower`, which refuses to start on
  a missing key. The node has to be startable with nothing but a machine profile, and its unsafe
  default — the output being on — is the one that is `false`.
- **A stopped tf cannot be reproduced in a test.** `tf2` keeps the latest sample for a time-zero
  lookup, so `test_watchdog.cpp` pins the missing lookup and the recovery from it instead; the
  deadline that actually protects a moving robot against a dead localizer is `cmd_timeout`, because
  the follower stops publishing once its own transform is gone.
