def endpoint_unavailable(endpoint: str) -> RuntimeError:
    return RuntimeError(
        f"{endpoint} unavailable. Keep './docker/container.sh visual' running "
        "in another host terminal and verify both shells use the same ROS_DOMAIN_ID. "
        "For an isolated check, run './docker/container.sh verify-demos' on the host."
    )


def servo_endpoint_unavailable(endpoint: str) -> RuntimeError:
    return RuntimeError(
        f"{endpoint} unavailable. Start fake hardware in one container shell, "
        "then start MoveIt in another container shell. "
        "Verify both shells use the same ROS_DOMAIN_ID. For an isolated check, "
        "run './docker/container.sh verify-demos' on the host."
    )
