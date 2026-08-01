from threading import Thread

from control_msgs.action import FollowJointTrajectory
from open_manipulator_demos.motion_validation import (
    JointStateMonitor,
    wait_for_joint_target,
)
from open_manipulator_demos.parameter_validation import (
    finite_sequence,
    positive_float,
    scaling_factor,
)
from open_manipulator_demos.robots import open_manipulator_x
from pymoveit2 import MoveIt2
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def _double_array_parameter(node: Node, name: str, expected_length: int) -> list[float]:
    values = list(node.get_parameter(name).get_parameter_value().double_array_value)
    return finite_sequence(name, values, expected_length)


def _move_to_configuration(
    *,
    node: Node,
    moveit2: MoveIt2,
    joint_state_monitor: JointStateMonitor,
    joint_positions: list[float],
    joint_tolerance: float,
    wait_timeout_sec: float,
    label: str,
) -> None:
    node.get_logger().info(f"zero-g {label}: joint_positions={joint_positions}")
    moveit2.move_to_configuration(joint_positions)
    if not moveit2.wait_until_executed():
        raise RuntimeError(f"zero-g {label} execution failed")
    actual_positions = wait_for_joint_target(
        monitor=joint_state_monitor,
        joint_names=open_manipulator_x.joint_names(),
        target_positions=joint_positions,
        tolerance=joint_tolerance,
        timeout_sec=wait_timeout_sec,
    )
    node.get_logger().info(
        f"zero-g {label} measured at target: joint_positions={actual_positions}"
    )


def main() -> None:
    rclpy.init()
    node = Node("demo_zero_g_practice")

    node.declare_parameter("motion_mode", "joint")
    node.declare_parameter("approach_joint_positions", [0.0, -0.45, 0.75, 0.20])
    node.declare_parameter("hold_joint_positions", [0.0, -0.55, 0.85, 0.20])
    node.declare_parameter("retreat_joint_positions", [0.0, -0.60, 0.90, 0.25])
    node.declare_parameter("object_position", [-0.08, -0.20, 0.205])
    node.declare_parameter("object_radius", 0.025)
    node.declare_parameter("max_velocity", 0.15)
    node.declare_parameter("max_acceleration", 0.15)
    node.declare_parameter("mirror_planning_scene", True)
    node.declare_parameter("setup_only", False)
    node.declare_parameter("wait_timeout_sec", 30.0)
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
    arm_controller_action = ActionClient(
        node,
        FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory",
        callback_group=callback_group,
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
        motion_mode = str(node.get_parameter("motion_mode").value).lower()
        if motion_mode != "joint":
            raise ValueError(
                'motion_mode must be "joint"; use ex_pose_goal for tested pose planning'
            )
        approach_joint_positions = _double_array_parameter(
            node, "approach_joint_positions", len(open_manipulator_x.joint_names())
        )
        hold_joint_positions = _double_array_parameter(
            node, "hold_joint_positions", len(open_manipulator_x.joint_names())
        )
        retreat_joint_positions = _double_array_parameter(
            node, "retreat_joint_positions", len(open_manipulator_x.joint_names())
        )
        object_position = _double_array_parameter(node, "object_position", 3)
        object_radius = positive_float(
            "object_radius", node.get_parameter("object_radius").value
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
            raise RuntimeError("/move_action server unavailable")
        if not arm_controller_action.wait_for_server(timeout_sec=wait_timeout_sec):
            raise RuntimeError("/arm_controller/follow_joint_trajectory unavailable")

        node.get_logger().info(
            "zero-g practice parameters: "
            f"motion_mode={motion_mode}, "
            f"approach_joint_positions={approach_joint_positions}, "
            f"hold_joint_positions={hold_joint_positions}, "
            f"retreat_joint_positions={retreat_joint_positions}, "
            f"object_position={object_position}, object_radius={object_radius}, "
            f"max_velocity={moveit2.max_velocity}, "
            f"max_acceleration={moveit2.max_acceleration}"
        )

        if bool(node.get_parameter("mirror_planning_scene").value):
            moveit2.add_collision_sphere(
                id="zero_g_floating_sample",
                position=object_position,
                radius=object_radius,
            )
            node.get_logger().info("zero-g floating sample mirrored in planning scene")

        if bool(node.get_parameter("setup_only").value):
            node.get_logger().info(
                "zero-g practice setup complete; no motion commanded"
            )
            return

        for label, joint_positions in [
            ("approach floating sample", approach_joint_positions),
            ("hold near floating sample", hold_joint_positions),
            ("retreat from floating sample", retreat_joint_positions),
        ]:
            _move_to_configuration(
                node=node,
                moveit2=moveit2,
                joint_state_monitor=joint_state_monitor,
                joint_positions=joint_positions,
                joint_tolerance=joint_tolerance,
                wait_timeout_sec=wait_timeout_sec,
                label=label,
            )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
