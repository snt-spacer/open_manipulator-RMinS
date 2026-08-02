from threading import Thread

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPlanningScene
from open_manipulator_demos.parameter_validation import (
    finite_sequence,
    non_empty_string,
    normalized_quaternion,
    positive_float,
)
from open_manipulator_demos.planning_scene_validation import wait_for_scene_object
from open_manipulator_demos.prerequisites import endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle


def _double_array_parameter(node: Node, name: str, expected_length: int) -> list[float]:
    values = list(node.get_parameter(name).get_parameter_value().double_array_value)
    return finite_sequence(name, values, expected_length)


def _example_mesh(scale: float) -> Mesh:
    vertices = [
        Point(x=0.0, y=0.0, z=0.0),
        Point(x=scale, y=0.0, z=0.0),
        Point(x=0.0, y=scale, z=0.0),
        Point(x=0.0, y=0.0, z=scale),
    ]
    triangles = [
        MeshTriangle(vertex_indices=[0, 1, 2]),
        MeshTriangle(vertex_indices=[0, 1, 3]),
        MeshTriangle(vertex_indices=[0, 2, 3]),
        MeshTriangle(vertex_indices=[1, 2, 3]),
    ]
    return Mesh(triangles=triangles, vertices=vertices)


def _pose(position: list[float], quat_xyzw: list[float]) -> Pose:
    return Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(
            x=quat_xyzw[0],
            y=quat_xyzw[1],
            z=quat_xyzw[2],
            w=quat_xyzw[3],
        ),
    )


def _collision_object(
    action: str,
    object_id: str,
    position: list[float],
    quat_xyzw: list[float],
    scale: float,
) -> CollisionObject:
    msg = CollisionObject()
    msg.header.frame_id = open_manipulator_x.base_link_name()
    msg.id = object_id
    if action == "remove":
        msg.operation = CollisionObject.REMOVE
        return msg

    msg.operation = CollisionObject.ADD
    msg.meshes.append(_example_mesh(scale))
    msg.mesh_poses.append(_pose(position, quat_xyzw))
    return msg


def main() -> None:
    rclpy.init()
    node = Node("ex_collision_mesh")

    node.declare_parameter("action", "add")
    node.declare_parameter("object_id", "example_mesh")
    node.declare_parameter("position", [0.18, 0.0, 0.12])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("scale", 0.04)
    node.declare_parameter("wait_timeout_sec", 10.0)

    planning_scene_client = node.create_client(GetPlanningScene, "get_planning_scene")
    collision_object_publisher = node.create_publisher(
        CollisionObject, "/collision_object", 10
    )

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        wait_timeout_sec = positive_float(
            "wait_timeout_sec", node.get_parameter("wait_timeout_sec").value
        )
        action = node.get_parameter("action").get_parameter_value().string_value.lower()
        if action not in {"add", "remove"}:
            raise ValueError("action must be one of: add, remove")
        object_id = non_empty_string(
            "object_id",
            node.get_parameter("object_id").get_parameter_value().string_value,
        )
        position = _double_array_parameter(node, "position", 3)
        quat_xyzw = normalized_quaternion(
            "quat_xyzw",
            node.get_parameter("quat_xyzw").get_parameter_value().double_array_value,
        )
        scale = positive_float("scale", node.get_parameter("scale").value)

        if not planning_scene_client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise endpoint_unavailable("get_planning_scene service")

        node.get_logger().info(
            "collision mesh parameters: "
            f"action={action}, object_id={object_id}, position={position}, "
            f"quat_xyzw={quat_xyzw}, scale={scale}"
        )

        collision_object_publisher.publish(
            _collision_object(action, object_id, position, quat_xyzw, scale)
        )
        wait_for_scene_object(
            node=node,
            planning_scene_client=planning_scene_client,
            object_id=object_id,
            present=action == "add",
            timeout_sec=wait_timeout_sec,
        )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
