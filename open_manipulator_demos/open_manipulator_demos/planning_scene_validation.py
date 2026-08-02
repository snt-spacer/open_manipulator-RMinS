import time

from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node


def planning_scene_object_ids(
    *, planning_scene_client, timeout_sec: float
) -> set[str]:
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )
    future = planning_scene_client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done():
        raise RuntimeError('timed out waiting for planning scene response')
    response = future.result()
    if response is None:
        raise RuntimeError('planning scene response was empty')
    return {obj.id for obj in response.scene.world.collision_objects}


def wait_for_scene_object(
    *,
    node: Node,
    planning_scene_client,
    object_id: str,
    present: bool,
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    object_ids: set[str] = set()
    while rclpy.ok() and time.monotonic() < deadline:
        object_ids = planning_scene_object_ids(
            planning_scene_client=planning_scene_client,
            timeout_sec=min(1.0, max(deadline - time.monotonic(), 0.001)),
        )
        if (object_id in object_ids) == present:
            state = 'present' if present else 'absent'
            node.get_logger().info(
                f'planning scene verified {object_id!r} is {state}'
            )
            return
        time.sleep(0.05)

    expected = 'present' if present else 'absent'
    raise RuntimeError(
        f'planning scene object {object_id!r} was not {expected}; '
        f'saw {sorted(object_ids)}'
    )
