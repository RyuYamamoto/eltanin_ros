# Observable navigation status. navyu published nothing comparable, so a failure was invisible from
# outside the process.
#
# This message is the single place where the STATE_* and OUTCOME_* constants are defined. rosidl has
# no mechanism for sharing constants between messages, so every other interface that carries an
# outcome documents that it uses eltanin_msgs/NavigationState's OUTCOME_* values, and C++ code refers
# to eltanin_msgs::msg::NavigationState::OUTCOME_*.

std_msgs/Header header

# The seven states of the navigator state machine (design section 6.7).
uint8 STATE_IDLE=0
uint8 STATE_PLANNING=1
uint8 STATE_FOLLOWING=2
uint8 STATE_REPLANNING=3
uint8 STATE_CANCELING=4
uint8 STATE_SUCCEEDED=5
uint8 STATE_FAILED=6
uint8 state

# 0..10 are eltanin's NavigateOutcome in declaration order, which is the wire format from here on:
# reordering the enum means updating the conversion table in eltanin_ros_common, not this file.
uint8 OUTCOME_REACHED=0
uint8 OUTCOME_MODEL_FAILED=1
uint8 OUTCOME_START_GOAL_FAILED=2
uint8 OUTCOME_PLAN_FAILED=3
uint8 OUTCOME_PATH_TOO_SHORT=4
uint8 OUTCOME_NO_PATH=5
uint8 OUTCOME_GOAL_TOLERANCE_FAILED=6
uint8 OUTCOME_REPLAN_FAILED=7
uint8 OUTCOME_REPLAN_LIMIT=8
uint8 OUTCOME_STALLED=9
uint8 OUTCOME_STEP_LIMIT=10
# 11..13 have no eltanin counterpart: they are outcomes only the ROS boundary can produce.
uint8 OUTCOME_CANCELED=11
uint8 OUTCOME_TIMEOUT=12
uint8 OUTCOME_INPUT_STALE=13
# Deliberately outside the enum's range, and the field default, so that a default-constructed
# message and an in-progress feedback do not read as OUTCOME_REACHED.
uint8 OUTCOME_UNKNOWN=255
uint8 outcome 255

string message
float64 distance_remaining
uint16 replans
