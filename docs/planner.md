# `eltanin_planner`

`global_path_planner` turns a goal into a path over the belief `global_costmap` publishes. It exists
because navyu could not say why a plan failed (N-5): `eltanin::planner::plan()` collapses four
distinct causes into one `nullopt`, and this node is where they are told apart again.

The design reasoning is in the Japanese design document
[`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md),
referenced by section number. Where the implementation diverged from it, the difference is
recorded under *Corrections to the design document* below rather than by editing the design
document.

## Why the action, and what its outcomes mean

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

## Telling the four causes apart costs a deliberate duplication

`attempt_plan()` runs the checks `Planner::plan()` runs — `world_to_map`, `classify(goal)`,
`find_nearest_traversable(start, r)` — **in the same order, with the same radius and the same model**,
before calling `plan()` itself. That duplication is intentional: giving `eltanin`'s `plan()` a
reason-carrying return value would change its API for the sake of the ROS boundary alone, and
`eltanin` is not modified from here. The consequence is a coupling worth knowing about: **if
`eltanin`'s `plan()` ever changes its checks, `attempt_plan()` has to follow**, or a real search
failure will be reported as one of the endpoint failures. `test_goal_validation.cpp` pins each
boundary, and the smoothing step is deliberately left outside `attempt_plan()` so that a
cancellation can be observed between the search and the smoother.

## Two searches, one set of failure classes

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

## Interruption is answered at four points, not during the search

A search cannot be interrupted, so a cancel or a preemption is observed only at four boundaries:
after acceptance, after the endpoints are resolved, after the search, and after the smoother. **A
cancel is therefore answered at worst one whole search late** — 0.147 s at `-O2`, 1.48 s at `-O0`.
A newer goal always wins: the running goal is aborted with `OUTCOME_CANCELED`, and a goal still
waiting is displaced the same way, since only one plan runs at a time and there is no queue. Planning
runs on a single worker thread rather than in the executor, so "at most one plan at a time" is a
property of this code and not of `std::thread::hardware_concurrency()`.

## Topics

| Topic | Type | QoS | Note |
|---|---|---|---|
| `global_costmap/global_costmap` (in) | `eltanin_msgs/Costmap` | `KeepLast(1)`, reliable, `transient_local` | Replaces the belief. Relative name; the launch file remaps it. |
| `global_costmap/global_costmap_updates` (in) | `eltanin_msgs/CostmapUpdate` | `KeepLast(1)`, reliable, volatile | Applied to the belief. |
| `~/global_path` (out) | `nav_msgs/Path` | `KeepLast(1)`, reliable, volatile | Smoothed. Published before `succeed()`, so a successful result always has its path on the topic too. |
| `~/global_path_raw` (out) | `nav_msgs/Path` | same | Unsmoothed; **the publisher only exists when `publish_raw_path` is set**. |
| `~/global_path_directed` (out) | `eltanin_msgs/DirectedPath` | same | The same poses as `~/global_path` plus how each segment is driven. Empty `segment_directions` means all forward. Always published; RViz cannot draw it, which is why `~/global_path` stays. |
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

## The goal is not trusted

navyu assumed every goal was already in the map frame (§2.4-2). Here `header.frame_id` is checked:
empty is a rejection rather than an assumption, a frame equal to `frames.map` skips tf entirely, and
anything else is transformed or rejected. The failing line names both frames, the stamp it looked
up, and the timeout it waited — because that is the information needed to fix it. **The requested
`orientation` is not discarded**: `plan()` puts `goal.yaw` on the last pose and the smoother leaves
the last pose's yaw alone, so the yaw that was asked for is the yaw that comes back.

## Parameters and startup

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

## Measured planning time

On a 4000x4000 empty map at `-O2`, `attempt_plan()` from cell (10, 10) to cell (3990, 3990) takes
**96 to 122 ms** over four runs and returns 3981 poses. That is in line with the 0.147 s `eltanin`
records for its own worst case, and it is what the 0.147 s figure in the cancellation discussion
above refers to. The measurement is `AttemptPlanTest.DISABLED_PlanScale`, kept disabled so CI never
runs it at `-O0`:

```bash
./build/eltanin_planner/test_goal_validation \
  --gtest_also_run_disabled_tests --gtest_filter=*PlanScale*
```

## Corrections to the design document

- §9 names `global_path_planner_node.cpp`. There is no such file and no hand-written `main`, for the
  same reason as [`global_costmap_node.cpp`](costmap.md#corrections-to-the-design-document).
- §3.3 says tf lookups use a timeout of 0. `global_path_planner` deviates: `tf_lookup_timeout`
  defaults to 0.1 s. The prohibition is about blocking inside a high-rate timer, and a plan is not
  periodic work — failing a plan because tf is one cycle behind is not useful. **The periodic
  consumers (tasks 11, 12, 15) keep a timeout of 0.** A consequence worth recording: under
  `use_sim_time` the timeout only expires while the simulator's clock is running.
- §6.3 calls the successful outcome "reached". In a planner it means a path was produced.
- §6.3 does not say that the pre-checks must track `eltanin`'s `plan()`. They must, in order, radius
  and model; see the duplication note above.
