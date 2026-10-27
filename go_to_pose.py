
#!/usr/bin/env python3

import math

import rclpy
from pymoveit2 import MoveIt2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread


# Robot geometry from the URDF
L1 = 0.075
L2 = 0.075

JOINT_1_LIMITS = (-2.09, 2.09)
JOINT_2_LIMITS = (-2.61, 2.61)
JOINT_Z_LIMITS = (-0.05, 0.05)

# joint_1 origin height relative to base_link
BASE_HEIGHT = 0.05


def within_limits(value, limits):
    return limits[0] <= value <= limits[1]




def inverse_kinematics(x, y, z, yaw_deg):
    """
    Solve SCARA IK for the requested XYZ position and yaw.
    Reject targets that are geometrically unreachable.
    """
    yaw = math.radians(yaw_deg)

    # The second link points along the requested yaw.
    vx = x - L2 * math.cos(yaw)
    vy = y - L2 * math.sin(yaw)

    # The remaining vector must have length L1.
    remaining_length = math.hypot(vx, vy)

    if abs(remaining_length - L1) > 1e-4:
        raise ValueError(
            "This XYZ position is incompatible with the requested yaw. "
            f"Required first-link length: {remaining_length:.4f} m; "
            f"actual length: {L1:.4f} m."
        )

    q1 = math.atan2(vy, vx)
    q2 = yaw - q1

    # Normalize q2 to [-pi, pi].
    q2 = math.atan2(math.sin(q2), math.cos(q2))

    if not within_limits(q1, JOINT_1_LIMITS):
        raise ValueError(
            f"joint_1={math.degrees(q1):.2f} deg is outside its limits."
        )

    if not within_limits(q2, JOINT_2_LIMITS):
        raise ValueError(
            f"joint_2={math.degrees(q2):.2f} deg is outside its limits."
        )

    qz = z - BASE_HEIGHT

    if not within_limits(qz, JOINT_Z_LIMITS):
        raise ValueError(
            f"Requested Z requires joint_z={qz:.4f} m, "
            "outside its limits."
        )

    return [q1, q2, qz]

class SCARATerminalController(Node):
    def __init__(self):
        super().__init__("scara_terminal_controller")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_z"],
            base_link_name="base_link",
            end_effector_name="z_quill",
            group_name="scara_arm",
            use_move_group_action=True,
        )

        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        self.get_logger().info(
            "SCARA joint-space terminal controller initialized."
        )

    def move_to_target(self, x, y, z, yaw_deg):
        try:
            joint_positions = inverse_kinematics(
                x, y, z, yaw_deg
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        q1, q2, qz = joint_positions

        self.get_logger().info(
            f"Joint target: joint_1={math.degrees(q1):.2f} deg, "
            f"joint_2={math.degrees(q2):.2f} deg, "
            f"joint_z={qz:.4f} m"
        )

        self.moveit2.move_to_configuration(
            joint_positions=joint_positions
        )

        completed = self.moveit2.wait_until_executed()

        if completed:
            self.get_logger().info("Joint-space motion succeeded.")
        else:
            self.get_logger().error(
                "Joint-space motion failed. Check MoveIt logs."
            )



def main():
    rclpy.init()
    node = SCARATerminalController()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True
    )
    executor_thread.start()

    try:
        while rclpy.ok():
            try:
                text = input(
                    "\nEnter X Y Z (m) and yaw (deg), or q to quit:\n> "
                ).strip()

                if text.lower() == "q":
                    break

                values = [float(v) for v in text.split()]

                if len(values) != 4 or not all(
                    math.isfinite(v) for v in values
                ):
                    print("Enter exactly four finite numbers.")
                    continue

                node.move_to_target(*values)

            except ValueError as exc:
                print(f"Invalid input: {exc}")

    except (KeyboardInterrupt, EOFError):
        pass

    finally:
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

