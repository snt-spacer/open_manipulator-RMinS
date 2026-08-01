#!/usr/bin/env python3

from __future__ import annotations

from threading import Condition, Thread
import math
import time

from control_msgs.action import FollowJointTrajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
DEFAULT_REST_POSITIONS = [
    0.0,
    -127.5 * np.pi / 180.0,
    87.5 * np.pi / 180.0,
    42.5 * np.pi / 180.0,
]  # radians
DEFAULT_DURATION_SEC = 4.0
DEFAULT_WAIT_TIMEOUT_SEC = 3.0


def _finite_duration(value: float, *, name: str) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return value


def _wait_for_future(future, *, timeout_sec: float, label: str):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)

    if not future.done():
        raise RuntimeError(f"{label} timed out after {timeout_sec:.1f} seconds")

    response = future.result()
    if response is None:
        raise RuntimeError(f"{label} returned no response")
    return response


class ShutdownLander(Node):
    def __init__(self) -> None:
        super().__init__("shutdown_lander")

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("rest_joint_positions", DEFAULT_REST_POSITIONS)
        self.declare_parameter("duration_sec", DEFAULT_DURATION_SEC)
        self.declare_parameter("num_points", 80)
        self.declare_parameter("epsilon", 0.02)
        self.declare_parameter(
            "action_topic", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("wait_timeout_sec", DEFAULT_WAIT_TIMEOUT_SEC)

        self.joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.rest_joint_positions = list(
            self.get_parameter("rest_joint_positions")
            .get_parameter_value()
            .double_array_value
        )
        self.duration_sec = _finite_duration(
            float(self.get_parameter("duration_sec").value), name="duration_sec"
        )
        self.num_points = int(self.get_parameter("num_points").value)
        self.epsilon = _finite_duration(
            float(self.get_parameter("epsilon").value), name="epsilon"
        )
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.wait_timeout_sec = _finite_duration(
            float(self.get_parameter("wait_timeout_sec").value),
            name="wait_timeout_sec",
        )

        if not self.joint_names:
            raise ValueError("joint_names cannot be empty")
        if len(self.joint_names) != len(self.rest_joint_positions):
            raise ValueError(
                "joint_names and rest_joint_positions must have the same length"
            )
        if self.num_points < 2:
            raise ValueError("num_points must be at least 2")

        self._state_condition = Condition()
        self._current_positions: list[float] | None = None
        self._joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_callback,
            10,
        )
        self._action_client = ActionClient(
            self, FollowJointTrajectory, self.action_topic
        )

    def _joint_state_callback(self, message: JointState) -> None:
        if not set(self.joint_names).issubset(set(message.name)):
            return

        positions = [
            message.position[message.name.index(joint)] for joint in self.joint_names
        ]
        with self._state_condition:
            self._current_positions = positions
            self._state_condition.notify_all()

    def _wait_for_joint_state(self) -> list[float]:
        deadline = time.monotonic() + self.wait_timeout_sec
        with self._state_condition:
            while rclpy.ok():
                if self._current_positions is not None:
                    return list(self._current_positions)

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("timed out waiting for joint states")
                self._state_condition.wait(timeout=min(remaining, 0.1))

        raise RuntimeError("ROS shutdown before joint states were available")

    def _create_smooth_trajectory(
        self, start_positions: list[float], end_positions: list[float]
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        times = np.linspace(0.0, self.duration_sec, self.num_points)
        for time_sec in times:
            time_norm = time_sec / self.duration_sec
            time_norm2 = time_norm * time_norm
            time_norm3 = time_norm2 * time_norm
            time_norm4 = time_norm3 * time_norm
            time_norm5 = time_norm4 * time_norm

            position_coeff = 10.0 * time_norm3 - 15.0 * time_norm4 + 6.0 * time_norm5
            velocity_coeff = (
                30.0 * time_norm2 - 60.0 * time_norm3 + 30.0 * time_norm4
            ) / self.duration_sec
            acceleration_coeff = (
                60.0 * time_norm - 180.0 * time_norm2 + 120.0 * time_norm3
            ) / (self.duration_sec * self.duration_sec)

            point = JointTrajectoryPoint()
            point.positions = [
                start + (end - start) * position_coeff
                for start, end in zip(start_positions, end_positions, strict=True)
            ]
            point.velocities = [
                (end - start) * velocity_coeff
                for start, end in zip(start_positions, end_positions, strict=True)
            ]
            point.accelerations = [
                (end - start) * acceleration_coeff
                for start, end in zip(start_positions, end_positions, strict=True)
            ]
            point.time_from_start.sec = int(time_sec)
            point.time_from_start.nanosec = int((time_sec % 1.0) * 1e9)
            trajectory.points.append(point)

        return trajectory

    def _wait_for_action_server(self) -> None:
        self.get_logger().info("Waiting for arm trajectory action server...")
        if not self._action_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            raise RuntimeError(f"{self.action_topic} unavailable")

    def lower_arm_to_rest(self) -> None:
        current_positions = self._wait_for_joint_state()
        deltas = [
            abs(current - target)
            for current, target in zip(
                current_positions, self.rest_joint_positions, strict=True
            )
        ]

        if max(deltas, default=0.0) <= self.epsilon:
            self.get_logger().info("Arm already near rest position; skipping lowering")
            return

        self._wait_for_action_server()
        self.get_logger().info(
            "Lowering arm to rest position: "
            f"current={current_positions}, target={self.rest_joint_positions}, "
            f"duration_sec={self.duration_sec}"
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._create_smooth_trajectory(
            current_positions, self.rest_joint_positions
        )
        goal.path_tolerance = []
        goal.goal_tolerance = []
        goal.goal_time_tolerance.sec = 0
        goal.goal_time_tolerance.nanosec = 0

        goal_future = self._action_client.send_goal_async(goal)
        goal_handle = _wait_for_future(
            goal_future,
            timeout_sec=self.wait_timeout_sec,
            label="trajectory goal response",
        )
        if not goal_handle.accepted:
            raise RuntimeError("rest-position trajectory goal was rejected")

        settle_sec = 0.5
        time.sleep(self.duration_sec + settle_sec)
        self.get_logger().info("Arm lowering command sent; proceeding to shutdown")


def main() -> None:
    rclpy.init()
    node = ShutdownLander()

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.lower_arm_to_rest()
    finally:
        rclpy.shutdown()
        executor_thread.join()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
