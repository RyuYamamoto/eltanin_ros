#!/usr/bin/env python3

# Copyright 2026 RyuYamamoto.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License

"""Forward an RViz goal pose to the eltanin global planner action."""

from action_msgs.msg import GoalStatus
from eltanin_msgs.action import ComputePathToPose
from eltanin_msgs.msg import NavigationState
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


PLANNER_ACTION = "compute_path_to_pose"


def make_plan_goal(pose: PoseStamped) -> ComputePathToPose.Goal:
    if not pose.header.frame_id:
        raise ValueError("goal frame_id must not be empty")

    goal = ComputePathToPose.Goal()
    goal.goal = pose
    goal.use_start = False
    return goal


class GoalPoseRelay(Node):

    def __init__(self):
        super().__init__("goal_pose_relay")
        self._planner = ActionClient(self, ComputePathToPose, PLANNER_ACTION)
        self._goal_subscription = self.create_subscription(
            PoseStamped, "goal_pose", self._on_goal_pose, 10
        )
        self.get_logger().info("waiting for a goal on %s" % self._goal_subscription.topic_name)

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        try:
            goal = make_plan_goal(pose)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        if not self._planner.server_is_ready():
            self.get_logger().error("planner action %s is not ready" % PLANNER_ACTION)
            return

        self.get_logger().info(
            "planning to (%.3f, %.3f) in %s"
            % (pose.pose.position.x, pose.pose.position.y, pose.header.frame_id)
        )
        future = self._planner.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy futures surface transport errors here.
            self.get_logger().error("failed to send planner goal: %s" % error)
            return

        if not goal_handle.accepted:
            self.get_logger().error("planner rejected the goal")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as error:  # rclpy futures surface transport errors here.
            self.get_logger().error("failed to receive planner result: %s" % error)
            return

        result = wrapped_result.result
        succeeded = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.outcome == NavigationState.OUTCOME_REACHED
        )
        if succeeded:
            self.get_logger().info("planned a path with %d poses" % len(result.path.poses))
            return

        self.get_logger().error(
            "planning failed: outcome=%d, message=%s" % (result.outcome, result.message)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
