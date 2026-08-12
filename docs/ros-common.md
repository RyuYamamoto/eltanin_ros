# `eltanin_ros_common`

The package has no nodes and builds two library targets. Which one a header belongs to is readable
from the header itself: one that includes `<rclcpp/...>` is a runtime piece.

The design reasoning is in the Japanese design document
[`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md),
referenced by section number. Where the implementation diverged from it, the difference is
recorded under *Corrections to the design document* below rather than by editing the design
document.

| Target | Headers | `rclcpp` |
|---|---|---|
| `eltanin_ros_common` | `conversion_result.hpp`, `cost_conversion.hpp`, `geometry_conversion.hpp`, `map_conversion.hpp`, `marker_conversion.hpp`, `path_conversion.hpp`, `scan_conversion.hpp`, `trajectory_conversion.hpp`, `warn_once.hpp` | no |
| `eltanin_ros_common_runtime` | `stale_input.hpp`, `timing.hpp`, `robot_profile.hpp` | yes |

Both are installed through one export set, so a downstream package gets them from a single
`find_package(eltanin_ros_common)`. The runtime target links the conversion layer publicly, so a node
that links the runtime target gets both; **the dependency never runs the other way.** The conversion
layer's `ldd` has no `librclcpp` in it and that is checked, not assumed. Only the tests that
construct a node call `rclcpp::init()`, and today that is one file, `test_robot_profile.cpp`.

## The conversion layer

Everything that crosses between `eltanin`'s types and ROS 2 messages is converted here and nowhere
else: cost value ranges, quaternions, twists, transforms, scans, maps, paths and trajectories.

`to_path()` is overloaded for `nav_msgs/Path`, for `eltanin_msgs/Trajectory2D` and for
`eltanin_msgs/DirectedPath`, so the caller picks by argument type rather than by name. The
`Trajectory2D` form **drops the magnitude of the velocity annotation but keeps its sign** as the
segment direction: `eltanin` has no trajectory type, so the speed has nowhere to go, but which way
the body drives does. There is no inverse, because nothing produces a `Trajectory2D` until
`local_path_planner` exists. `DirectedPath` converts both ways, and it is the only form that can
carry a reversing plan intact.

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

## What the conversions do and do not preserve

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

## Map origins are checked on three axes, not one

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

## Two places where `tf2` does not work the way it looks

- `tf2::Quaternion(0, 0, yaw)` **does not exist** in Jazzy. The three-argument constructor was
  removed, and in the old bullet API its argument order was `(yaw, pitch, roll)`, so that call would
  have meant `roll = yaw` — a mistake that compiles. Yaw is turned into a quaternion with
  `setRPY(0, 0, yaw)`; beware that `setEuler` takes yaw *first*.
- `tf2` only **declares** `fromMsg(const geometry_msgs::msg::Quaternion &, tf2::Quaternion &)`; the
  definition lives in `tf2_geometry_msgs`. So `tf2::getYaw(some_geometry_msgs_quaternion)` compiles
  against `tf2` alone but fails to link. `tf2_geometry_msgs` would drag in `tf2_ros` and therefore
  `rclcpp`, so this package builds the `tf2::Quaternion` from the four fields itself and calls
  `tf2::getYaw` / `tf2::getEulerYPR` on that.

## The runtime pieces

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

**There are no code defaults.** A profile that is missing a key stops the node with that key named,
which is the only way a different robot cannot inherit kachaka's collision box by accident. The
profiles ship from this package as `config/robot/<name>.yaml`, and every node package's tests start
their node from one, so an incomplete profile fails in CI. Write `robot.footprint` values
**with a decimal point**:
`[0, 0, 1, 0]` is an integer array to the parameter server and is rejected — with one line saying
so, not with a crash, but rejected.

`exact_footprint_check` is deliberately **not** a parameter. It is the fix for E-1 and a knob on it
is a knob on whether the robot notices obstacles; `test_exact_footprint_regression` pins both the
default and the behaviour from this side of the vendor boundary.

## Types whose first real user comes later

`WarnOnceLatch` and `StampedScan` have no node using them yet — the tests are their only callers.
They exist because the acceptance conditions they serve cannot be met otherwise: a node must not be
able to invent a `laser_frame`, so the scan conversion returns the frame and stamp together with the
`ScanData` (which carries neither), and a broken 2D assumption must not be silently dropped nor
logged once per cycle. First users are tasks 11, 12 and 15.

The same is true of the three runtime pieces: `RobotProfile` is first consumed by `global_costmap`
(task 6), `StaleInput` and `PeriodicClock` by `path_follower` and `collision_predictor` (tasks 11 and
12), and `StaleInput` again by `local_map` (task 15). If the shape does not fit its first real user,
change it there rather than working around it.
