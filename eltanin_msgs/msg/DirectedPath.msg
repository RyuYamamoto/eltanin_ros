# A geometric path whose segments carry the direction the body traverses them in. nav_msgs/Path
# cannot express this: the follower would have to guess it from the displacement and the body yaw,
# and the planner is the one that knows (eltanin docs/planner-design.md 14.15).
#
# poses[i].orientation is the BODY heading, not the tangent. On a reverse segment the direction of
# travel is that heading turned by pi.

std_msgs/Header header

geometry_msgs/Pose[] poses

# eltanin::Direction in declaration order, which is the wire format from here on.
uint8 DIRECTION_FORWARD=0
uint8 DIRECTION_REVERSE=1
uint8 DIRECTION_IN_PLACE=2
# One entry per segment: segment_directions[i] is how poses[i] -> poses[i+1] is driven.
# len(segment_directions) == len(poses) - 1, or 0 for a path that is entirely forward.
uint8[] segment_directions
