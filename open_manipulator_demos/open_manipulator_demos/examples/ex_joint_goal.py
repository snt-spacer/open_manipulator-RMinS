from threading import Thread

from open_manipulator_demos.motion_validation import (
    JointStateMonitor,
    wait_for_joint_target,
)
from open_manipulator_demos.parameter_validation import (
    finite_sequence,
    positive_float,
    scaling_factor,
)
from open_manipulator_demos.prerequisites import endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
from pymoveit2 import MoveIt2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def _double_array_parameter(node: Node, name: str, expected_length: int) -> list[float]:
    values = list(node.get_parameter(name).get_parameter_value().double_array_value)
    return finite_sequence(name, values, expected_length)


def main() -> None:
    rclpy.init()
    node = Node("ex_joint_goal")

    node.declare_parameter(
        "joint_positions", [0.0, -30 * 3.14159 / 180, 30 * 3.14159 / 180, 0.0]
    )
    node.declare_parameter("max_velocity", 0.25)
    node.declare_parameter("max_acceleration", 0.25)
    node.declare_parameter("wait_timeout_sec", 10.0)
    node.declare_parameter("joint_tolerance", 0.03)

    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=open_manipulator_x.joint_names(),
        base_link_name=open_manipulator_x.base_link_name(),
        end_effector_name=open_manipulator_x.end_effector_name(),
        group_name=open_manipulator_x.MOVE_GROUP_ARM,
        callback_group=callback_group,
        use_move_group_action=True,
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
        joint_positions = _double_array_parameter(
            node, "joint_positions", len(open_manipulator_x.joint_names())
        )
        moveit2.max_velocity = scaling_factor(
            "max_velocity", node.get_parameter("max_velocity").value
        )
        moveit2.max_acceleration = scaling_factor(
            "max_acceleration", node.get_parameter("max_acceleration").value
        )
        joint_tolerance = positive_float(
            "joint_tolerance", node.get_parameter("joint_tolerance").value
        )

        if not moveit2._MoveIt2__move_action_client.wait_for_server(
            timeout_sec=wait_timeout_sec
        ):
            raise endpoint_unavailable("/move_action server")

        node.get_logger().info(
            "joint goal parameters: "
            f"joint_positions={joint_positions}, "
            f"max_velocity={moveit2.max_velocity}, "
            f"max_acceleration={moveit2.max_acceleration}"
        )

        moveit2.move_to_configuration(joint_positions)
        if not moveit2.wait_until_executed():
            raise RuntimeError("joint goal execution failed")
        actual_positions = wait_for_joint_target(
            monitor=joint_state_monitor,
            joint_names=open_manipulator_x.joint_names(),
            target_positions=joint_positions,
            tolerance=joint_tolerance,
            timeout_sec=wait_timeout_sec,
        )
        node.get_logger().info(
            f"joint goal measured at target: joint_positions={actual_positions}"
        )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
