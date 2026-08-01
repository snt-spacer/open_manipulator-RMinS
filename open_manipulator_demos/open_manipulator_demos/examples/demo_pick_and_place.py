from threading import Thread
import time

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from open_manipulator_demos.motion_validation import (JointStateMonitor,
                                                      wait_for_joint_target)
from open_manipulator_demos.parameter_validation import (
    finite_sequence, positive_float, positive_sequence, scaling_factor)
from open_manipulator_demos.robots import open_manipulator_x
from pymoveit2 import MoveIt2
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

SCENE_OBJECT_IDS = {'tabletop_table', 'pickup_cube', 'place_zone'}


def _double_array_parameter(node: Node, name: str, expected_length: int) -> list[float]:
    values = list(node.get_parameter(name).get_parameter_value().double_array_value)
    return finite_sequence(name, values, expected_length)


def _scene_object_ids(
    *,
    planning_scene_client,
    timeout_sec: float,
) -> set[str]:
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )
    future = planning_scene_client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise RuntimeError('timed out waiting for planning scene response')
    result = future.result()
    if result is None:
        raise RuntimeError('planning scene response was empty')
    return {obj.id for obj in result.scene.world.collision_objects}


def _wait_for_scene_objects(
    *,
    node: Node,
    planning_scene_client,
    expected_ids: set[str],
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    last_ids: set[str] = set()
    while rclpy.ok() and time.monotonic() < deadline:
        last_ids = _scene_object_ids(
            planning_scene_client=planning_scene_client,
            timeout_sec=min(1.0, timeout_sec),
        )
        if expected_ids <= last_ids:
            node.get_logger().info(
                f'pick/place planning scene contains: {sorted(expected_ids)}'
            )
            return
        time.sleep(0.2)
    raise RuntimeError(
        'planning scene missing objects: '
        f'{sorted(expected_ids - last_ids)}; saw {sorted(last_ids)}'
    )


def _setup_planning_scene(
    *,
    node: Node,
    moveit2: MoveIt2,
    table_position: list[float],
    table_size: list[float],
    object_position: list[float],
    object_size: list[float],
    place_zone_position: list[float],
    place_zone_size: list[float],
) -> None:
    for object_id in SCENE_OBJECT_IDS:
        moveit2.remove_collision_object(id=object_id)
    time.sleep(0.3)

    moveit2.add_collision_box(
        id='tabletop_table',
        position=table_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=table_size,
    )
    moveit2.add_collision_box(
        id='pickup_cube',
        position=object_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=object_size,
    )
    moveit2.add_collision_box(
        id='place_zone',
        position=place_zone_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=place_zone_size,
    )
    node.get_logger().info(
        'pick/place scene objects added: tabletop_table, pickup_cube, place_zone'
    )


def main() -> None:
    rclpy.init()
    node = Node('demo_pick_and_place')

    node.declare_parameter('table_position', [-0.05, -0.18, 0.01])
    node.declare_parameter('table_size', [0.30, 0.24, 0.02])
    node.declare_parameter('object_position', [-0.08, -0.20, 0.035])
    node.declare_parameter('object_size', [0.025, 0.025, 0.025])
    node.declare_parameter('place_zone_position', [-0.13, -0.14, 0.023])
    node.declare_parameter('place_zone_size', [0.06, 0.06, 0.004])
    node.declare_parameter('pre_pick_joint_positions', [0.0, -0.45, 0.75, 0.20])
    node.declare_parameter('max_velocity', 0.15)
    node.declare_parameter('max_acceleration', 0.15)
    node.declare_parameter('setup_only', True)
    node.declare_parameter('wait_timeout_sec', 30.0)
    node.declare_parameter('joint_tolerance', 0.03)

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
        '/arm_controller/follow_joint_trajectory',
        callback_group=callback_group,
    )
    planning_scene_client = node.create_client(GetPlanningScene, 'get_planning_scene')
    joint_state_monitor = JointStateMonitor(node)

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        wait_timeout_sec = positive_float(
            'wait_timeout_sec', node.get_parameter('wait_timeout_sec').value
        )
        table_position = _double_array_parameter(node, 'table_position', 3)
        table_size = positive_sequence(
            'table_size',
            node.get_parameter('table_size').get_parameter_value().double_array_value,
            3,
        )
        object_position = _double_array_parameter(node, 'object_position', 3)
        object_size = positive_sequence(
            'object_size',
            node.get_parameter('object_size').get_parameter_value().double_array_value,
            3,
        )
        place_zone_position = _double_array_parameter(node, 'place_zone_position', 3)
        place_zone_size = positive_sequence(
            'place_zone_size',
            node.get_parameter(
                'place_zone_size'
            ).get_parameter_value().double_array_value,
            3,
        )
        pre_pick_joint_positions = _double_array_parameter(
            node, 'pre_pick_joint_positions', len(open_manipulator_x.joint_names())
        )
        moveit2.max_velocity = scaling_factor(
            'max_velocity', node.get_parameter('max_velocity').value
        )
        moveit2.max_acceleration = scaling_factor(
            'max_acceleration', node.get_parameter('max_acceleration').value
        )
        joint_tolerance = positive_float(
            'joint_tolerance', node.get_parameter('joint_tolerance').value
        )

        if not moveit2._MoveIt2__move_action_client.wait_for_server(
            timeout_sec=wait_timeout_sec
        ):
            raise RuntimeError('/move_action server unavailable')
        if not arm_controller_action.wait_for_server(timeout_sec=wait_timeout_sec):
            raise RuntimeError('/arm_controller/follow_joint_trajectory unavailable')
        if not planning_scene_client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise RuntimeError('get_planning_scene service unavailable')

        node.get_logger().info(
            'pick/place parameters: '
            f'table_position={table_position}, table_size={table_size}, '
            f'object_position={object_position}, object_size={object_size}, '
            f'place_zone_position={place_zone_position}, '
            f'place_zone_size={place_zone_size}, '
            f'pre_pick_joint_positions={pre_pick_joint_positions}, '
            f'max_velocity={moveit2.max_velocity}, '
            f'max_acceleration={moveit2.max_acceleration}'
        )

        _setup_planning_scene(
            node=node,
            moveit2=moveit2,
            table_position=table_position,
            table_size=table_size,
            object_position=object_position,
            object_size=object_size,
            place_zone_position=place_zone_position,
            place_zone_size=place_zone_size,
        )
        _wait_for_scene_objects(
            node=node,
            planning_scene_client=planning_scene_client,
            expected_ids=SCENE_OBJECT_IDS,
            timeout_sec=wait_timeout_sec,
        )

        if bool(node.get_parameter('setup_only').value):
            node.get_logger().info('pick/place setup complete; no motion commanded')
            return

        # Student exercise sequence:
        # 1. Open the gripper with the direct gripper example/interface.
        # 2. Move above the cube with direct MoveIt arm calls.
        # 3. Decide how to represent grasping in MoveIt before changing Gazebo.
        # 4. Move toward the place zone and update the planning scene explicitly.
        # This reference motion proves only that the arm can reach above the object.
        node.get_logger().info(
            'pick/place reference move above object: '
            f'joint_positions={pre_pick_joint_positions}'
        )
        moveit2.move_to_configuration(pre_pick_joint_positions)
        if not moveit2.wait_until_executed():
            raise RuntimeError('pick/place reference motion failed')
        actual_positions = wait_for_joint_target(
            monitor=joint_state_monitor,
            joint_names=open_manipulator_x.joint_names(),
            target_positions=pre_pick_joint_positions,
            tolerance=joint_tolerance,
            timeout_sec=wait_timeout_sec,
        )
        node.get_logger().info(
            'pick/place reference motion complete: '
            f'joint_positions={actual_positions}'
        )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
