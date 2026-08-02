from threading import Thread
import time

from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from open_manipulator_demos.motion_validation import (
    JointStateMonitor,
    wait_for_joint_displacement,
)
from open_manipulator_demos.parameter_validation import finite_float, positive_float
from open_manipulator_demos.prerequisites import servo_endpoint_unavailable
from open_manipulator_demos.robots import open_manipulator_x
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_srvs.srv import SetBool

SERVO_JOINT_TOPIC = "/servo_node/delta_joint_cmds"
SERVO_PAUSE_SERVICE = "/servo_node/pause_servo"
SERVO_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"


def _wait_for_service(
    node: Node, client, timeout_sec: float, service_name: str
) -> None:
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise servo_endpoint_unavailable(f"{service_name} service")


def _call_service(node: Node, client, request, timeout_sec: float, service_name: str):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done():
        raise RuntimeError(f"{service_name} service call timed out")

    response = future.result()
    if response is None:
        raise RuntimeError(f"{service_name} service call returned no response")
    return response


def _switch_command_type(
    node: Node, client, command_type: int, timeout_sec: float
) -> None:
    request = ServoCommandType.Request()
    request.command_type = command_type
    response = _call_service(
        node, client, request, timeout_sec, SERVO_COMMAND_TYPE_SERVICE
    )
    if not response.success:
        raise RuntimeError(
            f"{SERVO_COMMAND_TYPE_SERVICE} rejected command type: {response.message}"
        )


def _set_servo_paused(node: Node, client, paused: bool, timeout_sec: float) -> None:
    request = SetBool.Request()
    request.data = paused
    response = _call_service(
        node,
        client,
        request,
        timeout_sec,
        service_name=SERVO_PAUSE_SERVICE,
    )
    if not response.success:
        state = "pause" if paused else "unpause"
        raise RuntimeError(
            f"{SERVO_PAUSE_SERVICE} failed to {state}: {response.message}"
        )


def _make_joint_jog(
    *, node: Node, joint_name: str, joint_speed: float, duration_sec: float
) -> JointJog:
    jog = JointJog()
    jog.header.stamp = node.get_clock().now().to_msg()
    jog.joint_names = [joint_name]
    jog.velocities = [joint_speed]
    jog.duration = duration_sec
    return jog


def main() -> None:
    rclpy.init()
    node = Node("ex_servo")
    servo_unpaused = False
    wait_timeout_sec = 10.0

    node.declare_parameter("command_mode", "joint")
    node.declare_parameter("joint_name", "joint1")
    node.declare_parameter("joint_speed", 0.08)
    node.declare_parameter("frequency_hz", 20.0)
    node.declare_parameter("duration_sec", 2.0)
    node.declare_parameter("minimum_displacement", 0.01)
    node.declare_parameter("wait_timeout_sec", 10.0)

    command_qos = QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_ALL,
    )
    joint_publisher = node.create_publisher(JointJog, SERVO_JOINT_TOPIC, command_qos)
    joint_state_monitor = JointStateMonitor(node)
    command_type_client = node.create_client(
        ServoCommandType, SERVO_COMMAND_TYPE_SERVICE
    )
    pause_client = node.create_client(SetBool, SERVO_PAUSE_SERVICE)

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        wait_timeout_sec = positive_float(
            "wait_timeout_sec", node.get_parameter("wait_timeout_sec").value
        )
        command_mode = str(node.get_parameter("command_mode").value).lower()
        joint_name = str(node.get_parameter("joint_name").value)
        joint_speed = finite_float(
            "joint_speed", node.get_parameter("joint_speed").value
        )
        frequency_hz = positive_float(
            "frequency_hz", node.get_parameter("frequency_hz").value
        )
        duration_sec = positive_float(
            "duration_sec", node.get_parameter("duration_sec").value
        )
        minimum_displacement = positive_float(
            "minimum_displacement",
            node.get_parameter("minimum_displacement").value,
        )

        if command_mode != "joint":
            raise ValueError(
                'command_mode must be "joint"; Cartesian Servo commands are '
                "unsupported by the 4-DOF OpenMANIPULATOR-X arm"
            )
        if joint_name not in open_manipulator_x.joint_names():
            raise ValueError(
                f"joint_name must be one of: {open_manipulator_x.joint_names()}"
            )
        if joint_speed == 0.0:
            raise ValueError("joint_speed cannot be zero")

        frame_id = open_manipulator_x.base_link_name()
        node.get_logger().info(
            "servo parameters: "
            f"command_mode={command_mode}, joint_name={joint_name}, "
            f"joint_speed={joint_speed}, "
            f"frequency_hz={frequency_hz}, duration_sec={duration_sec}, "
            f"frame_id={frame_id}"
        )

        _wait_for_service(
            node, command_type_client, wait_timeout_sec, SERVO_COMMAND_TYPE_SERVICE
        )
        _wait_for_service(node, pause_client, wait_timeout_sec, SERVO_PAUSE_SERVICE)

        joint_names = open_manipulator_x.joint_names()
        initial_positions = joint_state_monitor.positions(
            joint_names, timeout_sec=wait_timeout_sec
        )
        _switch_command_type(
            node,
            command_type_client,
            ServoCommandType.Request.JOINT_JOG,
            wait_timeout_sec,
        )
        _set_servo_paused(node, pause_client, False, wait_timeout_sec)
        servo_unpaused = True

        period_sec = 1.0 / frequency_hz
        start_time = time.monotonic()
        while rclpy.ok():
            elapsed_sec = time.monotonic() - start_time
            if elapsed_sec >= duration_sec:
                break
            joint_publisher.publish(
                _make_joint_jog(
                    node=node,
                    joint_name=joint_name,
                    joint_speed=joint_speed,
                    duration_sec=period_sec,
                )
            )
            time.sleep(period_sec)

        actual_positions = wait_for_joint_displacement(
            monitor=joint_state_monitor,
            joint_names=joint_names,
            initial_positions=initial_positions,
            minimum_displacement=minimum_displacement,
            timeout_sec=wait_timeout_sec,
        )
        node.get_logger().info(
            "servo command stream moved the arm: "
            f"initial={initial_positions}, actual={actual_positions}"
        )
    finally:
        if servo_unpaused and rclpy.ok():
            try:
                _set_servo_paused(node, pause_client, True, wait_timeout_sec)
            except RuntimeError as exc:
                node.get_logger().warning(
                    f"failed to pause servo during shutdown: {exc}"
                )
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
