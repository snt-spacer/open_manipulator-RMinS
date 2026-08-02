from __future__ import annotations

from threading import Condition
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateMonitor:
    def __init__(self, node: Node) -> None:
        self._condition = Condition()
        self._positions: dict[str, float] = {}
        self._subscription = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

    def _joint_state_callback(self, message: JointState) -> None:
        with self._condition:
            self._positions.update(zip(message.name, message.position, strict=False))
            self._condition.notify_all()

    def positions(self, joint_names: list[str], timeout_sec: float) -> list[float]:
        if timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be positive')

        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while True:
                missing = [name for name in joint_names if name not in self._positions]
                if not missing:
                    return [self._positions[name] for name in joint_names]

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        f'timed out waiting for joint states: {sorted(missing)}'
                    )
                self._condition.wait(timeout=min(remaining, 0.1))


def wait_for_joint_target(
    *,
    monitor: JointStateMonitor,
    joint_names: list[str],
    target_positions: list[float],
    tolerance: float,
    timeout_sec: float,
) -> list[float]:
    if len(joint_names) != len(target_positions):
        raise ValueError('joint names and target positions must have the same length')
    if tolerance <= 0.0:
        raise ValueError('joint tolerance must be positive')

    deadline = time.monotonic() + timeout_sec
    actual_positions: list[float] = []
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        actual_positions = monitor.positions(joint_names, max(remaining, 0.001))
        errors = [
            abs(actual - target)
            for actual, target in zip(actual_positions, target_positions, strict=True)
        ]
        if max(errors, default=0.0) <= tolerance:
            return actual_positions
        time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

    errors = [
        abs(actual - target)
        for actual, target in zip(actual_positions, target_positions, strict=True)
    ]
    raise RuntimeError(
        'joint target was not reached: '
        f'target={target_positions}, actual={actual_positions}, errors={errors}, '
        f'tolerance={tolerance}'
    )


def wait_for_joint_displacement(
    *,
    monitor: JointStateMonitor,
    joint_names: list[str],
    initial_positions: list[float],
    minimum_displacement: float,
    timeout_sec: float,
) -> list[float]:
    if len(joint_names) != len(initial_positions):
        raise ValueError('joint names and initial positions must have the same length')
    if minimum_displacement <= 0.0:
        raise ValueError('minimum displacement must be positive')

    deadline = time.monotonic() + timeout_sec
    actual_positions: list[float] = []
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        actual_positions = monitor.positions(joint_names, max(remaining, 0.001))
        displacement = max(
            (
                abs(actual - initial)
                for actual, initial in zip(
                    actual_positions, initial_positions, strict=True
                )
            ),
            default=0.0,
        )
        if displacement >= minimum_displacement:
            return actual_positions
        time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

    raise RuntimeError(
        'servo command produced no measured joint motion: '
        f'initial={initial_positions}, actual={actual_positions}, '
        f'minimum_displacement={minimum_displacement}'
    )
