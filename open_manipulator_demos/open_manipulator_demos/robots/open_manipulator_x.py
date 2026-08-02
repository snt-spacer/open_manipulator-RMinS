MOVE_GROUP_ARM = "arm"
MOVE_GROUP_GRIPPER = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS = [0.018]
CLOSED_GRIPPER_JOINT_POSITIONS = [-0.004]


def joint_names() -> list[str]:
    return ["joint1", "joint2", "joint3", "joint4"]


def base_link_name() -> str:
    return "world"


def arm_base_link_name() -> str:
    return "link1"


def end_effector_name() -> str:
    return "end_effector_link"


def gripper_joint_names() -> list[str]:
    return ["gripper_left_joint"]
