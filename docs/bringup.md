# `eltanin_bringup`

Launch, parameters, RViz configuration and the tests that fence them, plus one script:
`goal_pose_relay`, which exists so that RViz's `2D Goal Pose` can start a plan. One launch file
exists so far, covering the global half of the stack — a map, `global_costmap` and
`global_path_planner`. Nothing here drives a robot: there is no controller and no `cmd_vel`
publisher in the stack yet, so this cannot move anything.

The design reasoning is in the Japanese design document
[`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md),
referenced by section number. Where the implementation diverged from it, the difference is
recorded under *Corrections to the design document* below rather than by editing the design
document.

## Running it, from a clean shell

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

**6. A goal.** Either press `2D Goal Pose` in RViz — `goal_pose_relay` turns that into an action
goal and logs the outcome — or send the action directly, which is the only way to choose
`use_start`. `use_start: false` reads the start from tf; pass `use_start: true` with a `start` pose
to plan from somewhere else.

```bash
ros2 action send_goal /global_path_planner/compute_path_to_pose \
  eltanin_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map},
           pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}},
    use_start: false}"
```

The result carries `outcome`, `message` and the path. `outcome: 0` is success; every other value is
explained in the [`eltanin_planner` outcome table](planner.md#why-the-action-and-what-its-outcomes-mean),
and `message` is the same single line the node logs.

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

## Arguments

| Argument | Default | Meaning |
|---|---|---|
| `robot_profile` | `kachaka` | Which `config/robot/<name>.yaml` the whole stack reads. `sim_robot` is the other one. |
| `use_sim_time` | `false` | Follow `/clock`. Pass `true` with the simulator; the planner's tf timeout only expires while that clock runs. |
| `use_composition` | `true` | One container, or one process per node. Both paths run the same registered component; the separate one sends 16 MB across a process boundary on every update and is for debugging. |
| `use_rviz` | `true` | Start RViz with `rviz/eltanin.rviz`. |
| `use_goal_pose_relay` | `true` | Start `goal_pose_relay`, which turns RViz's `2D Goal Pose` into a `compute_path_to_pose` goal. Off leaves the tool with nothing listening. |
| `map` | empty | Map yaml for `nav2_map_server`. **Empty means no `map_server` at all**, and the static map then comes from whoever else publishes `/map`. **Omit the argument to get that**; `map:=''` is not a way to say it, because the shell hands `ros2 launch` the literal `map:=` and it answers *malformed launch argument*. |
| `costmap_params_file` | `eltanin_costmap/config/global_costmap.param.yaml` | Every `global_costmap` key. The node has no defaults of its own. |
| `planner_params_file` | `eltanin_planner/config/global_path_planner.param.yaml` | Every `global_path_planner` key. The node has no defaults of its own. |
| `rviz_config` | `rviz/eltanin.rviz` | RViz configuration. |

Every argument is declared, every declared argument is read, and `test_launch_file.py` fails if
either stops being true. navyu's `localization.launch.py` read one it never declared, so passing it
changed nothing and `--show-args` did not list it.

**Parameters are written once.** `config/robot/*.yaml` uses the `/**` wildcard, so `robot.*` and
`frames.*` reach every node as the same number — which is what keeps `global_costmap`'s inflation
threshold and `global_path_planner`'s `Free` boundary in agreement. Each node's own config holds
only what is specific to one node, keyed by node name. No key appears in both files. navyu had four
copies of the same values and they had drifted apart.

## The static map has two ways in, and `map` picks between them

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
object and drops every accumulated observation with it, for the reason in
[Why accumulating state does not break the origin rule](costmap.md#why-accumulating-state-does-not-break-the-origin-rule).

## Composition is what makes the 16 MB affordable

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

## There is no localization yet, so tf comes from the command line

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

## What M2 asked for, and how to see it

| Condition | How to check |
|---|---|
| The static map is ingested both ways | Launch with `map:=<yaml>`; then launch with the argument omitted and run `map_server` by hand. `/global_costmap/global_costmap_visual` appears either way. |
| The global costmap is built and published | `ros2 topic echo --once /global_costmap/global_costmap --field header`. There is no rate to measure: the node has no timer. |
| A goal produces a path | The action result carries `outcome: 0` and the path, and `/global_path_planner/global_path` carries the same one. The last pose keeps the yaw that was asked for. |
| RViz shows map, costmap and path | The two `Map` displays and `GlobalPath`, with the transforms from step 4. The costmap uses the `costmap` colour scheme, without which the inflation is one flat shade. |
| An unreachable goal fails visibly | A goal inside a wall returns `outcome: 5` (`OUTCOME_NO_PATH`) with a `message`; a goal off the map returns `outcome: 2` (`OUTCOME_START_GOAL_FAILED`). Neither is a silent empty path, which is what navyu produced. |
| A frame mismatch is detected | A goal in `odom` is transformed and succeeds. A goal in a frame tf has never seen returns `outcome: 2`, and the log line names both frames, the stamp and the timeout. An empty `frame_id` is refused rather than assumed to be `map`. |

Run on navyu's 4000x4000 map with the transforms of step 4, `use_rviz:=false`, `rmw_zenoh_cpp`, and
the shipped `global_path_planner.param.yaml` — so `planner_type` is `hybrid_astar`. `map_server` read the map in
0.65 s and `global_costmap` built the whole area 0.33 s later; `lifecycle_bringup` drove both
transitions without a manager node. **The search times below are hybrid A\*'s**: the same goal under
`planner_type: astar` was measured at 68.5 ms, so a change to that parameter moves them by more than
an order of magnitude.

| Goal | Result |
|---|---|
| `(5.0, 3.0)` in `map`, `use_start: false` | `outcome: 0`, 157 poses, 11.038 m, searched in 1493 ms |
| the same with `use_composition:=false` | byte-identical plan, so neither start-up path changes the result |
| `(90, 90)`, unknown space | `outcome: 5` — *rejected goal cell (3799, 3799): cost 255 classifies as Inscribed, not Free; the goal is reported, not moved* |
| `(500, 0)`, off the map | `outcome: 2` — *it lies outside the 4000x4000 map at 0.050000 m from (-100.000000, -100.000000)* |
| `(3.0, 1.0)` in `odom` | `outcome: 0` — transformed, then planned |
| the same in `nowhere` | `outcome: 2` — *'nowhere' to frames.map 'map' at 0 ns is not available within 0.100000 s* |
| the same with an empty `frame_id` | `outcome: 2` — *its header.frame_id is empty; frames.map 'map' is not assumed* |
| `(4.0, -2.0)` published on `/goal_pose` | `goal_pose_relay` logs *planned a path with 111 poses*, which is the RViz `2D Goal Pose` path end to end |

`ros2 param get` on both nodes returns the same `robot.inflation_radius` and the same
`robot.footprint`, which is the property the whole parameter layout exists to guarantee.

One thing to know when checking by hand: `ros2 topic echo --once
/global_path_planner/global_path` after the plan has finished prints nothing, because that topic is
`volatile` on purpose (a late subscriber must not be handed an abandoned goal's path). Echo it in
another terminal before sending the goal, or read the path out of the action result.

## The fences

`colcon test` runs four pytest files over the configuration itself. They exist because none of what
they check fails at startup:

| Test | What it refuses |
|---|---|
| `test_robot_profile_config.py` | A profile missing a key (it would fall back to the C++ default and stop describing the machine), a footprint written with integer literals, and a kachaka profile that has drifted from `declare_robot_profile()`'s defaults or stopped being the collision box. |
| `test_navigation_config.py` | A node-name key no node answers to, a machine value written here as well as in `robot/*.yaml`, a number whose YAML spelling gives it the wrong parameter type, and smoother weights that diverge. |
| `test_rviz_config.py` | A display whose topic nothing publishes, a display whose publisher is switched off in the shipped node configs, the wrong colour scheme on the costmap, and a tool whose topic no node started by this launch file reads. |
| `test_launch_file.py` | An argument read but not declared, an argument declared but not read, the single-threaded container, a component without `use_intra_process_comms`, and the two start-up paths passing different parameters. |

The `test_rviz_config.py` gate is the one worth knowing about when adding a display: a topic that
only exists when a parameter is true has to have that parameter turned on in the same commit. That is
why `publish_footprint_path` is `true` in `global_path_planner.param.yaml` — `FootprintPath` is in the RViz
configuration, and a display of a topic that is never published is the navyu defect this fence names.

RViz's Map display logs one shader link error (`indexed_8bit_image.vert`, "active samplers with a
different type refer to the same texture image unit") on some drivers. It is an upstream rviz2
issue, not a problem with the configuration; the display is still created.

## The first plan after startup can come back INPUT_STALE

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

## Corrections to the design document

- §6.3 and the [`eltanin_planner` topic table](planner.md#topics) say the launch file remaps the costmap
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
