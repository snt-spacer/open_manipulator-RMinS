from math import sqrt
import time

from rclpy.time import Time
from tf2_ros import Buffer, TransformException


def wait_for_current_pose(
    *,
    transform_buffer: Buffer,
    frame_id: str,
    target_link: str,
    timeout_sec: float,
) -> tuple[list[float], list[float]]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            transform = transform_buffer.lookup_transform(
                frame_id, target_link, Time()
            ).transform
        except TransformException:
            time.sleep(0.05)
            continue

        return (
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ],
        )

    raise RuntimeError(
        f'transform {frame_id} -> {target_link} was unavailable for '
        f'{timeout_sec} seconds'
    )


def wait_for_position_target(
    *,
    transform_buffer: Buffer,
    frame_id: str,
    target_link: str,
    position: list[float],
    position_tolerance: float,
    timeout_sec: float,
) -> tuple[list[float], list[float]]:
    deadline = time.monotonic() + timeout_sec
    last_position: list[float] = []
    last_quaternion: list[float] = []
    while time.monotonic() < deadline:
        try:
            transform = transform_buffer.lookup_transform(
                frame_id, target_link, Time()
            ).transform
        except TransformException:
            time.sleep(0.05)
            continue

        last_position = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
        last_quaternion = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        position_error = sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(
                    last_position, position, strict=True
                )
            )
        )
        if position_error <= position_tolerance:
            return last_position, last_quaternion
        time.sleep(0.05)

    raise RuntimeError(
        'pose target was not reached: '
        f'target_position={position}, actual_position={last_position}'
    )
