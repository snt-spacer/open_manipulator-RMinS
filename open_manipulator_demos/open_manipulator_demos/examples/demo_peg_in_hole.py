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

SCENE_OBJECT_IDS = {
    'hole_module',
    'round_hole_target',
    'square_hole_target',
    'round_peg',
    'square_peg',
}


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
                f'peg/hole planning scene contains: {sorted(expected_ids)}'
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
    module_position: list[float],
    module_size: list[float],
    round_hole_position: list[float],
    square_hole_position: list[float],
    round_peg_position: list[float],
    square_peg_position: list[float],
    peg_height: float,
    round_peg_radius: float,
    square_peg_size: float,
) -> None:
    for object_id in SCENE_OBJECT_IDS:
        moveit2.remove_collision_object(id=object_id)
    time.sleep(0.3)

    moveit2.add_collision_box(
        id='hole_module',
        position=module_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=module_size,
    )
    moveit2.add_collision_cylinder(
        id='round_hole_target',
        position=round_hole_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        height=0.004,
        radius=round_peg_radius * 1.6,
    )
    moveit2.add_collision_box(
        id='square_hole_target',
        position=square_hole_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=[square_peg_size * 1.6, square_peg_size * 1.6, 0.004],
    )
    moveit2.add_collision_cylinder(
        id='round_peg',
        position=round_peg_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        height=peg_height,
        radius=round_peg_radius,
    )
    moveit2.add_collision_box(
        id='square_peg',
        position=square_peg_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=[square_peg_size, square_peg_size, peg_height],
    )
    node.get_logger().info(
        'peg/hole scene objects added: hole_module, round_hole_target, '
        'square_hole_target, round_peg, square_peg'
    )


def main() -> None:
    rclpy.init()
    node = Node('demo_peg_in_hole')

    node.declare_parameter('module_position', [-0.08, -0.18, 0.031])
    node.declare_parameter('module_size', [0.16, 0.11, 0.012])
    node.declare_parameter('round_hole_position', [-0.13, -0.21, 0.039])
    node.declare_parameter('square_hole_position', [-0.13, -0.15, 0.039])
    node.declare_parameter('round_peg_position', [-0.03, -0.21, 0.060])
    node.declare_parameter('square_peg_position', [-0.03, -0.15, 0.060])
    node.declare_parameter('peg_height', 0.042)
    node.declare_parameter('round_peg_radius', 0.007)
    node.declare_parameter('square_peg_size', 0.014)
    node.declare_parameter('approach_joint_positions', [0.0, -0.45, 0.75, 0.20])
    node.declare_parameter('max_velocity', 0.12)
    node.declare_parameter('max_acceleration', 0.12)
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
        module_position = _double_array_parameter(node, 'module_position', 3)
        module_size = positive_sequence(
            'module_size',
            node.get_parameter('module_size').get_parameter_value().double_array_value,
            3,
        )
        round_hole_position = _double_array_parameter(node, 'round_hole_position', 3)
        square_hole_position = _double_array_parameter(node, 'square_hole_position', 3)
        round_peg_position = _double_array_parameter(node, 'round_peg_position', 3)
        square_peg_position = _double_array_parameter(node, 'square_peg_position', 3)
        approach_joint_positions = _double_array_parameter(
            node, 'approach_joint_positions', len(open_manipulator_x.joint_names())
        )
        peg_height = positive_float(
            'peg_height', node.get_parameter('peg_height').value
        )
        round_peg_radius = positive_float(
            'round_peg_radius', node.get_parameter('round_peg_radius').value
        )
        square_peg_size = positive_float(
            'square_peg_size', node.get_parameter('square_peg_size').value
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
            'peg/hole parameters: '
            f'module_position={module_position}, module_size={module_size}, '
            f'round_hole_position={round_hole_position}, '
            f'square_hole_position={square_hole_position}, '
            f'round_peg_position={round_peg_position}, '
            f'square_peg_position={square_peg_position}, '
            f'peg_height={peg_height}, round_peg_radius={round_peg_radius}, '
            f'square_peg_size={square_peg_size}, '
            f'approach_joint_positions={approach_joint_positions}, '
            f'max_velocity={moveit2.max_velocity}, '
            f'max_acceleration={moveit2.max_acceleration}'
        )

        _setup_planning_scene(
            node=node,
            moveit2=moveit2,
            module_position=module_position,
            module_size=module_size,
            round_hole_position=round_hole_position,
            square_hole_position=square_hole_position,
            round_peg_position=round_peg_position,
            square_peg_position=square_peg_position,
            peg_height=peg_height,
            round_peg_radius=round_peg_radius,
            square_peg_size=square_peg_size,
        )
        _wait_for_scene_objects(
            node=node,
            planning_scene_client=planning_scene_client,
            expected_ids=SCENE_OBJECT_IDS,
            timeout_sec=wait_timeout_sec,
        )

        if bool(node.get_parameter('setup_only').value):
            node.get_logger().info('peg/hole setup complete; no motion commanded')
            return

        # Student exercise sequence:
        # 1. Inspect the module, pegs, and target markers in the planning scene.
        # 2. Move above a selected peg with direct MoveIt arm calls.
        # 3. Use the gripper example/interface for open and close commands.
        # 4. Represent the carried-object state with explicit MoveIt scene updates.
        # 5. Plan toward the target marker and check clearances before insertion.
        node.get_logger().info(
            'peg/hole reference move above peg: '
            f'joint_positions={approach_joint_positions}'
        )
        moveit2.move_to_configuration(approach_joint_positions)
        if not moveit2.wait_until_executed():
            raise RuntimeError('peg/hole reference motion failed')
        actual_positions = wait_for_joint_target(
            monitor=joint_state_monitor,
            joint_names=open_manipulator_x.joint_names(),
            target_positions=approach_joint_positions,
            tolerance=joint_tolerance,
            timeout_sec=wait_timeout_sec,
        )
        node.get_logger().info(
            'peg/hole reference motion complete: '
            f'joint_positions={actual_positions}'
        )
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
