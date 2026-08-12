# `eltanin_costmap`

One node so far, `global_costmap`. It owns the whole-area inflated costmap and, with it, the belief a
replan runs against. **Inflation lives here and nowhere else** in the stack.

The design reasoning is in the Japanese design document
[`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md),
referenced by section number. Where the implementation diverged from it, the difference is
recorded under *Corrections to the design document* below rather than by editing the design
document.

## It has no timer, and that is the design

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

## What "an unknown obstacle" means, and why it never goes away by itself

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

## Why accumulating state does not break the origin rule

`eltanin/docs/costmap-design.md` §14.2 rests "an origin change does not shift the cells" on there
being no layer that carries state across an update. Accumulating observations breaks that premise in
form. It does not break it here: **this costmap keeps the geometry of the static map for its whole
life and nothing calls `set_origin()` or `center_on()`** — a grep for either in `eltanin_costmap`
returns only the comment saying so. A stored cell index therefore means the same cell on every
update. A second `/map` message does not move the geometry either; it **replaces the whole object and
drops every accumulated cell**, because those indices would otherwise mean cells in a different grid.
Republishing the same map has that same effect, so do not use `/map` as a keep-alive.

## The patch, and the `⊕ r` in it

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

## Parameters and startup

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

## Corrections to the design document

Recorded here rather than by editing `docs/design/eltaninnavyuros.md`, same as for the conversions:

- §6.9 says `StaleInput::get(now)` returns `nullopt` when stale. It returns a pointer instead;
  `std::optional<T>` would copy a 16 MB costmap every cycle.
- §6.9 counts nine headers for this package. There are twelve public ones: the nine of the conversion
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
