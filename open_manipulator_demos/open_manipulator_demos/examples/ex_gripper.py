from threading import Thread

from open_manipulator_demos.motion_validation import (
    JointStateMonitor,
    wait_for_joint_target,
)
from open_manipulator_demos.parameter_validation import positive_float
from open_manipulator_demos.prerequisites import endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
from pymoveit2 import GripperCommand
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("ex_gripper")

    node.declare_parameter("action", "toggle")
    node.declare_parameter("wait_timeout_sec", 10.0)
    node.declare_parameter("joint_tolerance", 0.003)

    callback_group = ReentrantCallbackGroup()
    gripper = GripperCommand(
        node=node,
        gripper_joint_names=open_manipulator_x.gripper_joint_names(),
        open_gripper_joint_positions=open_manipulator_x.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=open_manipulator_x.CLOSED_GRIPPER_JOINT_POSITIONS,
        callback_group=callback_group,
        gripper_command_action_name="gripper_controller/gripper_cmd",
    )
    joint_state_monitor = JointStateMonitor(node)

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        wait_timeout_sec = positive_float(
            "wait_timeout_sec", node.get_parameter("wait_timeout_sec").value
        )
        joint_tolerance = positive_float(
            "joint_tolerance", node.get_parameter("joint_tolerance").value
        )
        action = node.get_parameter("action").get_parameter_value().string_value.lower()
        if action not in {"open", "close", "toggle"}:
            raise ValueError("action must be one of: open, close, toggle")

        if not gripper.gripper_command_action_client.wait_for_server(
            timeout_sec=wait_timeout_sec
        ):
            raise endpoint_unavailable("/gripper_controller/gripper_cmd action server")

        node.get_logger().info(f"gripper parameters: action={action}")

        resolved_action = action
        if action == "toggle":
            current_positions = joint_state_monitor.positions(
                open_manipulator_x.gripper_joint_names(),
                timeout_sec=wait_timeout_sec,
            )
            current = current_positions[0]
            opened = open_manipulator_x.OPEN_GRIPPER_JOINT_POSITIONS[0]
            closed = open_manipulator_x.CLOSED_GRIPPER_JOINT_POSITIONS[0]
            resolved_action = (
                "close" if abs(current - opened) <= abs(current - closed) else "open"
            )
            node.get_logger().info(
                f"gripper toggle resolved to {resolved_action}: current={current}"
            )

        if resolved_action == "open":
            gripper.open()
        else:
            gripper.close()

        if not gripper.wait_until_executed():
            raise RuntimeError("gripper execution failed")

        target_positions = (
            open_manipulator_x.OPEN_GRIPPER_JOINT_POSITIONS
            if resolved_action == "open"
            else open_manipulator_x.CLOSED_GRIPPER_JOINT_POSITIONS
        )
        actual_positions = wait_for_joint_target(
            monitor=joint_state_monitor,
            joint_names=open_manipulator_x.gripper_joint_names(),
            target_positions=target_positions,
            tolerance=joint_tolerance,
            timeout_sec=wait_timeout_sec,
        )
        node.get_logger().info(
            f"gripper measured at target: joint_positions={actual_positions}"
        )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
