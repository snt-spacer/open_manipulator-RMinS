from threading import Thread

from open_manipulator_demos.parameter_validation import positive_float
from open_manipulator_demos.prerequisites import endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
from pymoveit2 import GripperCommand, MoveIt2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("open_manipulator_x_template")

    node.declare_parameter("wait_timeout_sec", 10.0)
    wait_timeout_sec = positive_float(
        "wait_timeout_sec", node.get_parameter("wait_timeout_sec").value
    )

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
    gripper = GripperCommand(
        node=node,
        gripper_joint_names=open_manipulator_x.gripper_joint_names(),
        open_gripper_joint_positions=open_manipulator_x.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=open_manipulator_x.CLOSED_GRIPPER_JOINT_POSITIONS,
        callback_group=callback_group,
        gripper_command_action_name="gripper_controller/gripper_cmd",
    )

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        if not moveit2._MoveIt2__move_action_client.wait_for_server(
            timeout_sec=wait_timeout_sec
        ):
            raise endpoint_unavailable("/move_action server")
        if not gripper.gripper_command_action_client.wait_for_server(
            timeout_sec=wait_timeout_sec
        ):
            raise endpoint_unavailable("/gripper_controller/gripper_cmd action server")

        moveit2.max_velocity = 0.25
        moveit2.max_acceleration = 0.25

        node.get_logger().info("template ready for direct MoveIt commands")
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
