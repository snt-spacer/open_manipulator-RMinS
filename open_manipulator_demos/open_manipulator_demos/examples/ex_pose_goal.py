from threading import Thread

from open_manipulator_demos.parameter_validation import (
    finite_sequence,
    normalized_quaternion,
    positive_float,
    scaling_factor,
)
from open_manipulator_demos.pose_validation import (
    wait_for_current_pose,
    wait_for_position_target,
)
from open_manipulator_demos.prerequisites import endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
from pymoveit2 import MoveIt2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def _double_array_parameter(node: Node, name: str, expected_length: int) -> list[float]:
    values = list(node.get_parameter(name).get_parameter_value().double_array_value)
    return finite_sequence(name, values, expected_length)


def _validate_quaternion(quat_xyzw: list[float]) -> None:
    normalized_quaternion("quat_xyzw", quat_xyzw)


def main() -> None:
    rclpy.init()
    node = Node("ex_pose_goal")

    node.declare_parameter("position", [0.0, -0.042, 0.134])
    node.declare_parameter("quat_xyzw", [0.175, 0.175, -0.685, 0.685])
    node.declare_parameter("cartesian", False)
    node.declare_parameter("relative_cartesian", False)
    node.declare_parameter("position_tolerance", 0.02)
    node.declare_parameter("orientation_tolerance", 0.02)
    node.declare_parameter("planning_attempts", 3)
    node.declare_parameter("allowed_planning_time", 2.0)
    node.declare_parameter("max_velocity", 0.25)
    node.declare_parameter("max_acceleration", 0.25)
    node.declare_parameter("wait_timeout_sec", 10.0)

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
    transform_buffer = Buffer()
    transform_listener = TransformListener(transform_buffer, node)

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        wait_timeout_sec = positive_float(
            "wait_timeout_sec", node.get_parameter("wait_timeout_sec").value
        )
        position = _double_array_parameter(node, "position", 3)
        quat_xyzw = _double_array_parameter(node, "quat_xyzw", 4)
        _validate_quaternion(quat_xyzw)
        cartesian = bool(node.get_parameter("cartesian").value)
        relative_cartesian = bool(node.get_parameter("relative_cartesian").value)
        if relative_cartesian and not cartesian:
            raise ValueError("relative_cartesian requires cartesian:=true")
        position_tolerance = positive_float(
            "position_tolerance", node.get_parameter("position_tolerance").value
        )
        orientation_tolerance = positive_float(
            "orientation_tolerance",
            node.get_parameter("orientation_tolerance").value,
        )
        planning_attempts = int(node.get_parameter("planning_attempts").value)
        allowed_planning_time = positive_float(
            "allowed_planning_time",
            node.get_parameter("allowed_planning_time").value,
        )
        if planning_attempts < 1:
            raise ValueError("planning_attempts must be at least 1")
        moveit2.max_velocity = scaling_factor(
            "max_velocity", node.get_parameter("max_velocity").value
        )
        moveit2.max_acceleration = scaling_factor(
            "max_acceleration", node.get_parameter("max_acceleration").value
        )
        moveit2.allowed_planning_time = allowed_planning_time

        if not moveit2._MoveIt2__move_action_client.wait_for_server(
            timeout_sec=wait_timeout_sec
        ):
            raise endpoint_unavailable("/move_action server")

        if relative_cartesian:
            offset = position
            current_position, current_quaternion = wait_for_current_pose(
                transform_buffer=transform_buffer,
                frame_id=open_manipulator_x.base_link_name(),
                target_link=open_manipulator_x.end_effector_name(),
                timeout_sec=wait_timeout_sec,
            )
            position = [
                current + delta
                for current, delta in zip(current_position, offset, strict=True)
            ]
            quat_xyzw = current_quaternion
            node.get_logger().info(
                "resolved relative Cartesian target: "
                f"offset={offset}, start_position={current_position}, "
                f"target_position={position}"
            )

        node.get_logger().info(
            "pose goal parameters: "
            f"position={position}, quat_xyzw={quat_xyzw}, cartesian={cartesian}, "
            f"relative_cartesian={relative_cartesian}, "
            f"position_tolerance={position_tolerance}, "
            f"orientation_tolerance={orientation_tolerance}, "
            f"planning_attempts={planning_attempts}, "
            f"allowed_planning_time={moveit2.allowed_planning_time}, "
            f"max_velocity={moveit2.max_velocity}, "
            f"max_acceleration={moveit2.max_acceleration}"
        )

        last_validation_error: RuntimeError | None = None
        for attempt in range(1, planning_attempts + 1):
            moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                target_link=open_manipulator_x.end_effector_name(),
                frame_id=open_manipulator_x.base_link_name(),
                tolerance_position=position_tolerance,
                tolerance_orientation=orientation_tolerance,
                cartesian=cartesian,
            )
            if moveit2.wait_until_executed():
                try:
                    actual_position, actual_quaternion = wait_for_position_target(
                        transform_buffer=transform_buffer,
                        frame_id=open_manipulator_x.base_link_name(),
                        target_link=open_manipulator_x.end_effector_name(),
                        position=position,
                        position_tolerance=position_tolerance,
                        timeout_sec=wait_timeout_sec,
                    )
                except RuntimeError as error:
                    last_validation_error = error
                    node.get_logger().warning(
                        f"pose goal attempt {attempt}/{planning_attempts} "
                        f"failed measured-position validation: {error}"
                    )
                else:
                    node.get_logger().info(
                        "pose goal measured at target position: "
                        f"position={actual_position}, "
                        f"quat_xyzw={actual_quaternion}"
                    )
                    break
                continue
            node.get_logger().warning(
                f"pose goal attempt {attempt}/{planning_attempts} failed"
            )
        else:
            validation_detail = (
                f"; last validation error: {last_validation_error}"
                if last_validation_error is not None
                else ""
            )
            raise RuntimeError(
                f"pose goal execution failed after {planning_attempts} attempts"
                f"{validation_detail}"
            )
    finally:
        transform_listener.unregister()
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
