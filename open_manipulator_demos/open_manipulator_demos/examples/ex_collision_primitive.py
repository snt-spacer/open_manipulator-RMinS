from threading import Thread

from moveit_msgs.srv import GetPlanningScene
from open_manipulator_demos.parameter_validation import (
    finite_sequence,
    normalized_quaternion,
    positive_float,
    positive_sequence,
)
from open_manipulator_demos.planning_scene_validation import wait_for_scene_object
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


def _dimensions_for_shape(shape: str, dimensions: list[float]) -> list[float]:
    expected_lengths = {
        "box": 3,
        "sphere": 1,
        "cylinder": 2,
        "cone": 2,
    }
    if shape not in expected_lengths:
        raise ValueError("shape must be one of: box, sphere, cylinder, cone")
    expected_length = expected_lengths[shape]
    return positive_sequence(f"{shape} dimensions", dimensions, expected_length)


def main() -> None:
    rclpy.init()
    node = Node("ex_collision_primitive")

    node.declare_parameter("shape", "box")
    node.declare_parameter("action", "add")
    node.declare_parameter("object_id", "")
    node.declare_parameter("position", [0.18, 0.0, 0.12])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("dimensions", [0.04, 0.04, 0.04])
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
    planning_scene_client = node.create_client(GetPlanningScene, "get_planning_scene")

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        wait_timeout_sec = positive_float(
            "wait_timeout_sec", node.get_parameter("wait_timeout_sec").value
        )
        shape = node.get_parameter("shape").get_parameter_value().string_value.lower()
        action = node.get_parameter("action").get_parameter_value().string_value.lower()
        if action not in {"add", "remove"}:
            raise ValueError("action must be one of: add, remove")
        object_id = node.get_parameter("object_id").get_parameter_value().string_value
        object_id = object_id or f"{shape}_primitive"
        position = _double_array_parameter(node, "position", 3)
        quat_xyzw = normalized_quaternion(
            "quat_xyzw",
            node.get_parameter("quat_xyzw").get_parameter_value().double_array_value,
        )
        dimensions = list(
            node.get_parameter("dimensions").get_parameter_value().double_array_value
        )
        dimensions = _dimensions_for_shape(shape, dimensions)

        if not planning_scene_client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise endpoint_unavailable("get_planning_scene service")

        node.get_logger().info(
            "collision primitive parameters: "
            f"action={action}, shape={shape}, object_id={object_id}, "
            f"position={position}, quat_xyzw={quat_xyzw}, dimensions={dimensions}"
        )

        if action == "remove":
            moveit2.remove_collision_object(id=object_id)
            wait_for_scene_object(
                node=node,
                planning_scene_client=planning_scene_client,
                object_id=object_id,
                present=False,
                timeout_sec=wait_timeout_sec,
            )
            return

        if shape == "box":
            moveit2.add_collision_box(
                id=object_id,
                position=position,
                quat_xyzw=quat_xyzw,
                size=dimensions,
            )
        elif shape == "sphere":
            moveit2.add_collision_sphere(
                id=object_id,
                position=position,
                radius=dimensions[0],
            )
        elif shape == "cylinder":
            moveit2.add_collision_cylinder(
                id=object_id,
                position=position,
                quat_xyzw=quat_xyzw,
                height=dimensions[0],
                radius=dimensions[1],
            )
        else:
            moveit2.add_collision_cone(
                id=object_id,
                position=position,
                quat_xyzw=quat_xyzw,
                height=dimensions[0],
                radius=dimensions[1],
            )
        wait_for_scene_object(
            node=node,
            planning_scene_client=planning_scene_client,
            object_id=object_id,
            present=True,
            timeout_sec=wait_timeout_sec,
        )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
