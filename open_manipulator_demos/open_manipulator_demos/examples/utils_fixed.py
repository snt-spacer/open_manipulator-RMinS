from math import acos, cos, radians, sin, sqrt
import time
from typing import Any

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PlanningScene,
                             PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from open_manipulator_demos.parameter_validation import (
    finite_float, non_empty_string, positive_float, scaling_factor)
from open_manipulator_demos.prerequisites import endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


class PegInHole(Node):
    """Synchronous teaching helpers for OpenMANIPULATOR-X exercises."""

    def __init__(self, wait_timeout_sec: float = 10.0) -> None:
        super().__init__('peg_in_hole')
        self.wait_timeout_sec = positive_float(
            'wait_timeout_sec', wait_timeout_sec
        )
        self.speed = 0.9
        self.acceleration = 0.9
        self._joint_positions: dict[str, float] = {}

        self._move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
        )
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )
        self._get_scene_client = self.create_client(
            GetPlanningScene,
            '/get_planning_scene',
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )
        self._transform_buffer = Buffer()
        self._transform_listener = TransformListener(
            self._transform_buffer,
            self,
            spin_thread=False,
        )

        if not self._move_action_client.wait_for_server(
            timeout_sec=self.wait_timeout_sec
        ):
            raise endpoint_unavailable('/move_action server')
        self.get_logger().info('Peg-in-hole helpers connected to MoveIt')

    def set_speed(self, speed: float) -> None:
        self.speed = self._validate_scale('speed', speed)

    def set_acc(self, acceleration: float) -> None:
        self.acceleration = self._validate_scale('acceleration', acceleration)

    def wait(self, seconds: float) -> None:
        seconds = finite_float('seconds', seconds)
        if seconds < 0.0:
            raise ValueError('seconds cannot be negative')
        time.sleep(seconds)

    def move_gripper_to(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> bool:
        """Move the tool in the link1 frame, optionally constraining attitude."""
        try:
            target_position = [
                finite_float('x', x),
                finite_float('y', y),
                finite_float('z', z),
            ]
            orientation_values = (roll, pitch, yaw)
            supplied = [value is not None for value in orientation_values]
            if any(supplied) and not all(supplied):
                raise ValueError('roll, pitch, and yaw must be supplied together')

            orientation = None
            if all(supplied):
                orientation = self._quaternion_from_euler(
                    finite_float('roll', roll),
                    finite_float('pitch', pitch),
                    finite_float('yaw', yaw),
                )

            constraints = Constraints()
            constraints.name = 'goal_constraints'

            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = (
                open_manipulator_x.arm_base_link_name()
            )
            position_constraint.link_name = (
                open_manipulator_x.end_effector_name()
            )
            position_constraint.weight = 1.0

            target_region = SolidPrimitive()
            target_region.type = SolidPrimitive.SPHERE
            target_region.dimensions = [0.008]

            target_pose = Pose()
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            target_pose.orientation.w = 1.0
            position_constraint.constraint_region.primitives.append(
                target_region
            )
            position_constraint.constraint_region.primitive_poses.append(
                target_pose
            )
            constraints.position_constraints.append(position_constraint)

            if orientation is not None:
                orientation_constraint = OrientationConstraint()
                orientation_constraint.header.frame_id = (
                    open_manipulator_x.arm_base_link_name()
                )
                orientation_constraint.link_name = (
                    open_manipulator_x.end_effector_name()
                )
                orientation_constraint.orientation = orientation
                orientation_constraint.absolute_x_axis_tolerance = 0.1
                orientation_constraint.absolute_y_axis_tolerance = 0.1
                orientation_constraint.absolute_z_axis_tolerance = 0.1
                orientation_constraint.weight = 1.0
                constraints.orientation_constraints.append(
                    orientation_constraint
                )

            goal = self._new_move_goal()
            goal.request.goal_constraints.append(constraints)
            goal.request.num_planning_attempts = (
                20 if orientation is not None else 10
            )
            self._execute_move_goal(goal)
            actual_position = self._wait_for_pose_target(
                target_position=target_position,
                target_orientation=orientation,
                position_tolerance=0.012,
                orientation_tolerance=0.12,
            )
            self.get_logger().info(
                f'tool measured at target: position={actual_position}'
            )
            return True
        except Exception as exc:
            self.get_logger().error(f'tool move failed: {exc}')
            return False

    def move_gripper_joints(
        self,
        joint1: float,
        joint2: float,
        joint3: float,
        joint4: float,
    ) -> bool:
        """Move the four arm joints; input angles are in degrees."""
        try:
            target_positions = [
                radians(finite_float('joint1', joint1)),
                radians(finite_float('joint2', joint2)),
                radians(finite_float('joint3', joint3)),
                radians(finite_float('joint4', joint4)),
            ]
            constraints = Constraints()
            constraints.name = 'joint_goal_constraints'
            for name, position in zip(
                open_manipulator_x.joint_names(),
                target_positions,
                strict=True,
            ):
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = position
                constraint.tolerance_above = 0.01
                constraint.tolerance_below = 0.01
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)

            goal = self._new_move_goal()
            goal.request.goal_constraints.append(constraints)
            goal.request.num_planning_attempts = 5
            self._execute_move_goal(goal)
            actual_positions = self._wait_for_joint_target(
                joint_names=open_manipulator_x.joint_names(),
                target_positions=target_positions,
                tolerance=0.03,
            )
            self.get_logger().info(
                'arm joints measured at target: '
                f'joint_positions={actual_positions}'
            )
            return True
        except Exception as exc:
            self.get_logger().error(f'arm joint move failed: {exc}')
            return False

    def open_gripper(self) -> bool:
        return self.set_gripper_position(
            open_manipulator_x.OPEN_GRIPPER_JOINT_POSITIONS[0]
        )

    def close_gripper(self) -> bool:
        return self.set_gripper_position(
            open_manipulator_x.CLOSED_GRIPPER_JOINT_POSITIONS[0]
        )

    def set_gripper_position(
        self,
        position: float,
        max_effort: float = 10.0,
    ) -> bool:
        try:
            closed = open_manipulator_x.CLOSED_GRIPPER_JOINT_POSITIONS[0]
            opened = open_manipulator_x.OPEN_GRIPPER_JOINT_POSITIONS[0]
            target = finite_float('position', position)
            if not closed <= target <= opened:
                raise ValueError(
                    f'position must be between {closed} and {opened}'
                )
            max_effort = positive_float('max_effort', max_effort)
            if not self._gripper_action_client.wait_for_server(
                timeout_sec=self.wait_timeout_sec
            ):
                raise endpoint_unavailable(
                    '/gripper_controller/gripper_cmd action server'
                )

            goal = GripperCommand.Goal()
            goal.command = GripperCommandMsg()
            goal.command.position = target
            goal.command.max_effort = max_effort
            goal_handle = self._wait_for_future(
                self._gripper_action_client.send_goal_async(goal),
                'gripper goal response',
            )
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError('gripper goal was rejected')
            result = self._wait_for_future(
                goal_handle.get_result_async(),
                'gripper execution',
            )
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                status = None if result is None else result.status
                raise RuntimeError(
                    f'gripper execution failed with status {status}'
                )

            actual_positions = self._wait_for_joint_target(
                joint_names=open_manipulator_x.gripper_joint_names(),
                target_positions=[target],
                tolerance=0.002,
            )
            self.get_logger().info(
                'gripper measured at target: '
                f'joint_positions={actual_positions}'
            )
            return True
        except Exception as exc:
            self.get_logger().error(f'gripper command failed: {exc}')
            return False

    def spawn_cylinder(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        radius: float = 0.012,
        height: float = 0.05,
    ) -> bool:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]
        return self._spawn_primitive(
            name=name,
            x=x,
            y=y,
            z=z,
            primitive=primitive,
            z_offset=height / 2.0,
            dimensions=[radius, height],
        )

    def spawn_cube(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        size: float = 0.024,
    ) -> bool:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size, size, size]
        return self._spawn_primitive(
            name=name,
            x=x,
            y=y,
            z=z,
            primitive=primitive,
            z_offset=size / 2.0,
            dimensions=[size],
        )

    def attach_to_gripper(self, object_name: str) -> bool:
        try:
            object_name = non_empty_string('object_name', object_name)
            attached_object = AttachedCollisionObject()
            attached_object.link_name = 'link5'
            attached_object.object.id = object_name
            attached_object.object.operation = CollisionObject.ADD
            attached_object.touch_links = [
                'link5',
                'gripper_left_link',
                'gripper_right_link',
            ]

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.append(
                attached_object
            )
            self._apply_planning_scene(scene)
            self._wait_for_attached_object(object_name, present=True)
            self._wait_for_world_object(object_name, present=False)
            self.get_logger().info(f'attached {object_name!r} to link5')
            return True
        except Exception as exc:
            self.get_logger().error(f'object attachment failed: {exc}')
            return False

    def detach_from_gripper(self, object_name: str) -> bool:
        try:
            self._detach_object(object_name)
            self.get_logger().info(f'detached {object_name!r} from link5')
            return True
        except Exception as exc:
            self.get_logger().error(f'object detachment failed: {exc}')
            return False

    def detach_all(self) -> bool:
        try:
            _, attached_ids = self._scene_object_ids()
            for object_name in attached_ids:
                self._detach_object(object_name)
            self.get_logger().info('detached all objects from link5')
            return True
        except Exception as exc:
            self.get_logger().error(f'detaching all objects failed: {exc}')
            return False

    def current_gripper_position(
        self,
        frame_id: str | None = None,
    ) -> list[float]:
        frame = (
            non_empty_string('frame_id', frame_id)
            if frame_id is not None
            else open_manipulator_x.arm_base_link_name()
        )
        deadline = time.monotonic() + self.wait_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self._transform_buffer.lookup_transform(
                    frame,
                    open_manipulator_x.end_effector_name(),
                    Time(),
                ).transform
            except TransformException:
                continue
            return [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ]
        raise RuntimeError(
            f'timed out waiting for tool transform in frame {frame!r}'
        )

    @staticmethod
    def _validate_scale(name: str, value: float) -> float:
        return scaling_factor(name, value)

    def _new_move_goal(self) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        request = goal.request
        request.workspace_parameters.header.frame_id = (
            open_manipulator_x.arm_base_link_name()
        )
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        request.group_name = open_manipulator_x.MOVE_GROUP_ARM
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = self.speed
        request.max_acceleration_scaling_factor = self.acceleration
        goal.planning_options.plan_only = False
        return goal

    def _execute_move_goal(self, goal: MoveGroup.Goal) -> None:
        goal_handle = self._wait_for_future(
            self._move_action_client.send_goal_async(goal),
            'MoveIt goal response',
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt goal was rejected')
        result = self._wait_for_future(
            goal_handle.get_result_async(),
            'MoveIt execution',
            timeout_sec=max(self.wait_timeout_sec, 30.0),
        )
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if result is None else result.status
            raise RuntimeError(f'MoveIt execution failed with status {status}')
        error_code = result.result.error_code
        if error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                'MoveIt reported error '
                f'{error_code.val}: {error_code.message or "no details"}'
            )

    def _wait_for_future(
        self,
        future,
        description: str,
        timeout_sec: float | None = None,
    ) -> Any:
        timeout = self.wait_timeout_sec if timeout_sec is None else timeout_sec
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.1, max(remaining, 0.0)))
        if not future.done():
            raise RuntimeError(f'timed out waiting for {description}')
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f'{description} failed: {exception}') from exception
        return future.result()

    def _joint_state_callback(self, message: JointState) -> None:
        self._joint_positions.update(
            zip(message.name, message.position, strict=False)
        )

    def _wait_for_joint_target(
        self,
        *,
        joint_names: list[str],
        target_positions: list[float],
        tolerance: float,
    ) -> list[float]:
        deadline = time.monotonic() + self.wait_timeout_sec
        actual_positions: list[float] = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not all(name in self._joint_positions for name in joint_names):
                continue
            actual_positions = [
                self._joint_positions[name] for name in joint_names
            ]
            errors = [
                abs(actual - target)
                for actual, target in zip(
                    actual_positions,
                    target_positions,
                    strict=True,
                )
            ]
            if max(errors, default=0.0) <= tolerance:
                return actual_positions
        raise RuntimeError(
            'joint target was not reached: '
            f'target={target_positions}, actual={actual_positions}, '
            f'tolerance={tolerance}'
        )

    def _wait_for_pose_target(
        self,
        *,
        target_position: list[float],
        target_orientation: Quaternion | None,
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> list[float]:
        deadline = time.monotonic() + self.wait_timeout_sec
        actual_position: list[float] = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self._transform_buffer.lookup_transform(
                    open_manipulator_x.arm_base_link_name(),
                    open_manipulator_x.end_effector_name(),
                    Time(),
                ).transform
            except TransformException:
                continue

            actual_position = [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ]
            position_error = sqrt(
                sum(
                    (actual - target) ** 2
                    for actual, target in zip(
                        actual_position,
                        target_position,
                        strict=True,
                    )
                )
            )
            if position_error > position_tolerance:
                continue
            if target_orientation is None:
                return actual_position

            actual_orientation = transform.rotation
            if (
                self._quaternion_error(actual_orientation, target_orientation)
                <= orientation_tolerance
            ):
                return actual_position

        raise RuntimeError(
            'tool pose target was not reached: '
            f'target={target_position}, actual={actual_position}'
        )

    def _spawn_primitive(
        self,
        *,
        name: str,
        x: float,
        y: float,
        z: float,
        primitive: SolidPrimitive,
        z_offset: float,
        dimensions: list[float],
    ) -> bool:
        try:
            name = non_empty_string('name', name)
            for dimension in dimensions:
                positive_float('primitive dimension', dimension)
            x = finite_float('x', x)
            y = finite_float('y', y)
            z = finite_float('z', z)
            z_offset = finite_float('z_offset', z_offset)

            collision_object = CollisionObject()
            collision_object.header.frame_id = (
                open_manipulator_x.base_link_name()
            )
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = name
            collision_object.operation = CollisionObject.ADD
            collision_object.primitives.append(primitive)

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z) + z_offset
            pose.orientation.w = 1.0
            collision_object.primitive_poses.append(pose)

            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects.append(collision_object)
            self._apply_planning_scene(scene)
            self._wait_for_world_object(name, present=True)
            self.get_logger().info(f'planning-scene object {name!r} added')
            return True
        except Exception as exc:
            self.get_logger().error(f'object creation failed: {exc}')
            return False

    def _apply_planning_scene(self, scene: PlanningScene) -> None:
        if not self._apply_scene_client.wait_for_service(
            timeout_sec=self.wait_timeout_sec
        ):
            raise endpoint_unavailable('/apply_planning_scene service')
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait_for_future(
            self._apply_scene_client.call_async(request),
            'apply planning scene response',
        )
        if response is None or not response.success:
            raise RuntimeError('MoveIt rejected planning-scene update')

    def _scene_object_ids(self) -> tuple[set[str], set[str]]:
        if not self._get_scene_client.wait_for_service(
            timeout_sec=self.wait_timeout_sec
        ):
            raise endpoint_unavailable('/get_planning_scene service')
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        response = self._wait_for_future(
            self._get_scene_client.call_async(request),
            'get planning scene response',
        )
        if response is None:
            raise RuntimeError('planning-scene response was empty')
        world_ids = {
            obj.id for obj in response.scene.world.collision_objects
        }
        attached_ids = {
            obj.object.id
            for obj in response.scene.robot_state.attached_collision_objects
        }
        return world_ids, attached_ids

    def _wait_for_world_object(self, object_name: str, present: bool) -> None:
        deadline = time.monotonic() + self.wait_timeout_sec
        world_ids: set[str] = set()
        while rclpy.ok() and time.monotonic() < deadline:
            world_ids, _ = self._scene_object_ids()
            if (object_name in world_ids) == present:
                return
            time.sleep(0.05)
        expected = 'present' if present else 'absent'
        raise RuntimeError(
            f'world object {object_name!r} was not {expected}; '
            f'saw {sorted(world_ids)}'
        )

    def _wait_for_attached_object(
        self,
        object_name: str,
        present: bool,
    ) -> None:
        deadline = time.monotonic() + self.wait_timeout_sec
        attached_ids: set[str] = set()
        while rclpy.ok() and time.monotonic() < deadline:
            _, attached_ids = self._scene_object_ids()
            if (object_name in attached_ids) == present:
                return
            time.sleep(0.05)
        expected = 'attached' if present else 'detached'
        raise RuntimeError(
            f'object {object_name!r} was not {expected}; '
            f'saw {sorted(attached_ids)}'
        )

    def _detach_object(self, object_name: str) -> None:
        object_name = non_empty_string('object_name', object_name)
        attached_object = AttachedCollisionObject()
        attached_object.link_name = 'link5'
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached_object)
        self._apply_planning_scene(scene)
        self._wait_for_attached_object(object_name, present=False)

    @staticmethod
    def _quaternion_from_euler(
        roll: float,
        pitch: float,
        yaw: float,
    ) -> Quaternion:
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        quaternion = Quaternion()
        quaternion.w = cy * cp * cr + sy * sp * sr
        quaternion.x = cy * cp * sr - sy * sp * cr
        quaternion.y = sy * cp * sr + cy * sp * cr
        quaternion.z = sy * cp * cr - cy * sp * sr
        return quaternion

    @staticmethod
    def _quaternion_error(actual, target: Quaternion) -> float:
        actual_values = [actual.x, actual.y, actual.z, actual.w]
        target_values = [target.x, target.y, target.z, target.w]
        actual_norm = sqrt(sum(value * value for value in actual_values))
        target_norm = sqrt(sum(value * value for value in target_values))
        if actual_norm == 0.0 or target_norm == 0.0:
            raise ValueError('quaternion norm cannot be zero')
        dot = abs(
            sum(
                actual_value * target_value
                for actual_value, target_value in zip(
                    actual_values,
                    target_values,
                    strict=True,
                )
            )
            / (actual_norm * target_norm)
        )
        return 2.0 * acos(min(1.0, max(-1.0, dot)))


def _require_success(success: bool, operation: str) -> None:
    if not success:
        raise RuntimeError(f'{operation} failed')


def main() -> None:
    """Exercise the compatibility helper against the running ROS stack."""
    rclpy.init()
    helper = None
    try:
        helper = PegInHole(wait_timeout_sec=15.0)
        helper.set_speed(0.25)
        helper.set_acc(0.25)

        _require_success(
            helper.move_gripper_joints(0.0, -30.0, 30.0, 0.0),
            'utility arm motion',
        )
        current_position = helper.current_gripper_position()
        _require_success(
            helper.move_gripper_to(*current_position),
            'utility pose motion',
        )
        _require_success(helper.close_gripper(), 'utility gripper close')
        _require_success(helper.open_gripper(), 'utility gripper open')

        _require_success(
            helper.spawn_cylinder(
                'utils_fixed_cylinder',
                0.35,
                0.35,
                0.0,
                radius=0.006,
                height=0.02,
            ),
            'utility cylinder creation',
        )
        tool_world = helper.current_gripper_position(frame_id='world')
        cube_size = 0.008
        _require_success(
            helper.spawn_cube(
                'utils_fixed_cube',
                tool_world[0],
                tool_world[1],
                tool_world[2] - cube_size / 2.0,
                size=cube_size,
            ),
            'utility cube creation',
        )
        _require_success(
            helper.attach_to_gripper('utils_fixed_cube'),
            'utility object attachment',
        )
        _require_success(
            helper.detach_from_gripper('utils_fixed_cube'),
            'utility object detachment',
        )
        _require_success(helper.detach_all(), 'utility detach-all')
        helper.get_logger().info('utils_fixed workflow passed')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if helper is not None:
            helper.destroy_node()


if __name__ == '__main__':
    main()
