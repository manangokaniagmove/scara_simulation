#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint


class TestJointMove(Node):
    def __init__(self):
        super().__init__('test_joint_move')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def move(self):
        self.get_logger().info('Waiting for MoveIt...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveIt action server unavailable')
            return

        request = MotionPlanRequest()
        request.group_name = 'scara_arm'
        request.pipeline_id = 'ompl'
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 5

        constraints = Constraints()

        # Small, safe change from the current zero joint position
        joint = JointConstraint()
        joint.joint_name = 'joint_1'
        joint.position = 0.10
        joint.tolerance_above = 0.03
        joint.tolerance_below = 0.03
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)

        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted; waiting for result...')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result and result.result.error_code.val == 1:
            self.get_logger().info('SUCCESS: joint-space move completed')
        elif result:
            self.get_logger().error(
                f'MoveIt failed: {result.result.error_code.val}'
            )
        else:
            self.get_logger().error('No result received')


def main():
    rclpy.init()
    node = TestJointMove()
    try:
        node.move()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
