# Why the path follower is commanding what it is commanding. Published every cycle, including the
# cycles where the command is zero, so that a follower that stopped is never silent.
#
# Not to be confused with eltanin::control::FollowerState, which the eltanin side is introducing as
# the *input* to follow(): a pose and a measured twist. This message is the diagnostic that comes
# out, not the state that goes in.

std_msgs/Header header

# eltanin::control::PurePursuit::Status in declaration order, which is the wire format from here on:
# reordering the enum means updating the table in eltanin_controller, not just this file.
uint8 STATUS_NO_PATH=0
uint8 STATUS_TRACKING=1
uint8 STATUS_GOAL_REACHED=2
uint8 status

# eltanin::control::GoalApproach::State, same rule.
uint8 APPROACH_INACTIVE=0
uint8 APPROACH_APPROACHING=1
uint8 APPROACH_ALIGNING=2
uint8 APPROACH_REACHED=3
uint8 APPROACH_ALIGNMENT_TIMEOUT=4
uint8 approach_state

# Why the command is what it is. Anything but REASON_NONE means eltanin was either not called at all
# this cycle or called and did not return a tracking command; status and approach_state alone cannot
# express a failure on the input side.
uint8 REASON_NONE=0
uint8 REASON_NO_INPUT=1        # no path has ever arrived
uint8 REASON_INPUT_STALE=2     # the deadline on the path elapsed
uint8 REASON_INPUT_EMPTY=3     # the path carries no poses
uint8 REASON_INPUT_REJECTED=4  # wrong frame_id, or the conversion refused it
uint8 REASON_NO_TRANSFORM=5    # frames.map to frames.base could not be looked up
uint8 REASON_NO_DT=6           # no usable elapsed time this cycle, the first one included
uint8 REASON_CONTROLLER=7      # eltanin returned something other than tracking; read the two above
uint8 reason

# False only for the failures the follower cannot recover from on its own, which is the navigator's
# rule for a stopping trigger: AlignmentTimeout, PurePursuit::NoPath and a PurePursuit::GoalReached
# that beat GoalApproach to the goal. Every input-side failure leaves this true, because the
# upstream coming back fixes it.
bool ok

string message

# GoalApproach::Result as it is. A cycle that never reached eltanin reports +inf / +inf / 0, which is
# the vocabulary GoalApproach itself uses for an empty path.
float64 remaining_arc
float64 position_error
float64 yaw_error

# PurePursuit::Result::target_index; 0 unless status is STATUS_TRACKING.
uint32 lookahead_index

# The elapsed time handed to eltanin this cycle [s]; 0 on a cycle that did not call it. The upper
# bound the in-place alignment converges under is well inside a 20 Hz period, so this is the number
# to watch rather than the nominal update_frequency.
float64 control_dt

# GoalApproach::Result::align_elapsed [s], so that yaw_align_timeout - align_elapsed is readable
# from outside before the timeout fires rather than only after it.
float64 align_elapsed
