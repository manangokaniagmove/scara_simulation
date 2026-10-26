
#!/usr/bin/env python3

import math
import sys
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter

from pymoveit2 import MoveIt2
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
)


def main() -> int:
    rclpy.init()

    node = None
    moveit2 = None
    executor = None
    executor_thread = None
    status = 1

    try:
        node = Node("scara_terminal_pose")

        # SCARA target supplied through terminal prompts
        x = float(input("X (metres): "))
        y = float(input("Y (metres): "))
        z = float(input("Z (metres): "))
        yaw_deg = float(input("Yaw (degrees): "))

        yaw = math.radians(yaw_deg)

        # Convert yaw to quaternion (x, y, z, w)
        quat_xyzw = [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        ]
        position = [x, y, z]

        callback_group = ReentrantCallbackGroup()
        declare_robot_parameters(node)

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(
            target=executor.spin,
            daemon=True,
        )
        executor_thread.start()

        robot = RobotConfiguration(
            node,
            callback_group=callback_group,
        )

        moveit2 = MoveIt2(
            node=node,
            callback_group=callback_group,
            **robot.moveit2_kwargs(),
        )

        # Conservative speed for simulation tests
        moveit2.max_velocity = 0.2
        moveit2.max_acceleration = 0.2

        node.get_logger().info(
            f"Target position: {position}, yaw: {yaw_deg:.2f} degrees"
        )

        # First use normal MoveIt pose planning, not Cartesian interpolation
        submitted = moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=False,
            timeout_sec=30.0,
        )

        if not submitted:
            node.get_logger().error("Failed to submit pose goal.")
        elif moveit2.wait_until_executed(timeout_sec=30.0):
            node.get_logger().info("SCARA pose motion completed successfully.")
            status = 0
        else:
            node.get_logger().error(
                "Pose motion failed or timed out. Check MoveIt logs."
            )

    except ValueError:
        print("Invalid input. Enter numerical X, Y, Z and yaw values.")
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"{type(error).__name__}: {error}"
            )
        else:
            print(
                f"{type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        cleanup(
            moveit2,
            executor,
            executor_thread,
            "SCARA pose-goal",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        )

    return status


if __name__ == "__main__":
    sys.exit(main())
