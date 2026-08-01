# OpenMANIPULATOR-X demos

This package contains direct `pymoveit2` examples and Gazebo exercises for OpenMANIPULATOR-X. Use the repository's Docker environment described in the [student guide](../README.md).

Each direct example is a ROS client. It needs fake hardware or Gazebo, active controllers, and MoveIt running in another terminal.

## Run the complete check

From the repository root on the host:

```bash
./docker/container.sh verify-demos
```

The verifier starts and stops its own ROS stacks. It checks repeated arm goals, a short Cartesian waypoint, gripper state changes, Servo in both joint directions, every supported collision shape, custom mesh parameters, common invalid inputs, missing-endpoint guidance, setup-only launches, and Gazebo motion against measured state.

Do not keep another exercise running in the same container. The verifier may stop it while preparing an isolated test.

A successful run ends with:

```text
Direct examples and all Gazebo motion workflows passed.
```

## Run direct examples interactively

In the first host terminal, start fake hardware and MoveIt:

```bash
OPEN_MANIPULATOR_START_RVIZ=false ./docker/container.sh visual
```

Keep that command running. In the second host terminal, enter the same container:

```bash
./docker/container.sh enter
```

Check the required endpoints from the container shell:

```bash
ros2 action list
ros2 service list
```

The action list must contain `/move_action` and `/gripper_controller/gripper_cmd`. The service list must contain `/get_planning_scene` and `/apply_planning_scene`.

The following examples work with the `visual` stack:

| Command                                                  | Operation                                                                     |
| -------------------------------------------------------- | ----------------------------------------------------------------------------- |
| `ros2 run open_manipulator_demos template`               | Check MoveIt and gripper endpoints, then report readiness for direct commands |
| `ros2 run open_manipulator_demos ex_joint_goal`          | Move the four arm joints and check `/joint_states`                            |
| `ros2 run open_manipulator_demos ex_pose_goal`           | Plan a tool pose and check the measured position                              |
| `ros2 run open_manipulator_demos ex_gripper`             | Toggle the gripper and check its joint state                                  |
| `ros2 run open_manipulator_demos ex_utils_fixed`         | Exercise the synchronous teaching helper                                      |
| `ros2 run open_manipulator_demos ex_collision_primitive` | Add a primitive to the planning scene                                         |
| `ros2 run open_manipulator_demos ex_collision_mesh`      | Add a mesh to the planning scene                                              |

`ex_servo` needs the separate Servo startup described under [MoveIt Servo](#moveit-servo).

### ROS parameter syntax

Pass example parameters after `--ros-args`. Use `-p name:=value` for each value:

```bash
ros2 run open_manipulator_demos ex_gripper \
  --ros-args -p action:="open"
```

Strings can be quoted. Lists must use ROS list syntax inside quotes:

```bash
ros2 run open_manipulator_demos ex_pose_goal --ros-args \
  -p position:="[0.0, -0.042, 0.134]" \
  -p quat_xyzw:="[0.175, 0.175, -0.685, 0.685]"
```

Positions use metres in the `world` frame. Quaternions use `xyzw` order and must be normalized.

MoveIt uses position-only IK for this 4-DOF arm. The example verifies the measured tool position after motion and reports the measured quaternion. It does not claim that the requested orientation was reached.

Set `cartesian:=true` only for a short waypoint near the current tool pose. Normal planning can reach targets that have no complete straight-line path.

Set `relative_cartesian:=true` together with `cartesian:=true` to treat `position` as a short `[x, y, z]` offset from the measured tool pose. The request uses the current tool orientation, but position-only IK still verifies only the final position:

```bash
ros2 run open_manipulator_demos ex_pose_goal --ros-args \
  -p cartesian:=true \
  -p relative_cartesian:=true \
  -p position:='[0.006, 0.0, 0.0]' \
  -p position_tolerance:=0.003
```

### Common parameters

| Parameter          | Used by                        | Meaning                                                         |
| ------------------ | ------------------------------ | --------------------------------------------------------------- |
| `wait_timeout_sec` | Direct examples                | Maximum seconds to wait for an endpoint or result               |
| `max_velocity`     | Joint and pose goals           | MoveIt velocity scale in the range `(0, 1]`                     |
| `max_acceleration` | Joint and pose goals           | MoveIt acceleration scale in the range `(0, 1]`                 |
| `action`           | Gripper and collision examples | Operation such as `open`, `close`, `toggle`, `add`, or `remove` |

Parameter defaults are declared near the start of each file under `open_manipulator_demos/open_manipulator_demos/examples`.

## Use the synchronous helper

`open_manipulator_demos.examples.utils_fixed.PegInHole` provides blocking methods for short course scripts. Create it only after MoveIt's `/move_action` server is available.

```python
import rclpy
from open_manipulator_demos.examples.utils_fixed import PegInHole

rclpy.init()
robot = PegInHole(wait_timeout_sec=15.0)
try:
    robot.set_speed(0.25)
    robot.set_acc(0.25)
    if not robot.move_gripper_joints(0.0, -30.0, 30.0, 0.0):
        raise RuntimeError('arm motion failed')
finally:
    robot.destroy_node()
    rclpy.shutdown()
```

Joint-angle arguments use degrees. Tool targets use metres in `link1`; spawned objects use `world`. `current_gripper_position` uses `link1` unless it receives another frame.

| Method                                         | Result                                                 |
| ---------------------------------------------- | ------------------------------------------------------ |
| `set_speed(scale)`                             | Set the velocity scale in `(0, 1]`                     |
| `set_acc(scale)`                               | Set the acceleration scale in `(0, 1]`                 |
| `move_gripper_joints(j1, j2, j3, j4)`          | Move the four arm joints using degree inputs           |
| `move_gripper_to(x, y, z, roll, pitch, yaw)`   | Move the tool, with optional Euler angles in radians   |
| `open_gripper()` and `close_gripper()`         | Move the gripper to its configured limit               |
| `set_gripper_position(position, max_effort)`   | Move the gripper joint to a bounded radian position    |
| `spawn_cube(...)` and `spawn_cylinder(...)`    | Add an object and wait for planning-scene confirmation |
| `attach_to_gripper(name)`                      | Attach a known scene object to `link5`                 |
| `detach_from_gripper(name)` and `detach_all()` | Return attached objects to the world scene             |
| `current_gripper_position(frame_id)`           | Return the measured tool position as `[x, y, z]`       |

Motion and scene methods return `False` after logging a failure. Invalid speed, acceleration, delay, and constructor arguments raise an exception. `current_gripper_position` also raises if its transform times out.

Run the helper's complete arm, pose, gripper, scene, attach, and detach check against the active stack:

```bash
ros2 run open_manipulator_demos ex_utils_fixed
```

## Run Gazebo exercises

Run these launch commands from a container shell opened with `./docker/container.sh enter`. Start only one at a time.

```bash
ros2 launch open_manipulator_demos demo_pick_and_place.launch.py \
  setup_only:=true start_rviz:=false headless:=true

ros2 launch open_manipulator_demos demo_peg_in_hole.launch.py \
  setup_only:=true start_rviz:=false headless:=true

ros2 launch open_manipulator_demos demo_zero_g_practice.launch.py \
  setup_only:=false start_rviz:=false headless:=true
```

Setup mode starts Gazebo, controllers, and MoveIt, then creates the exercise scene without moving the arm.

| Argument                 | Meaning                                                   |
| ------------------------ | --------------------------------------------------------- |
| `headless:=false`        | Open the Gazebo client                                    |
| `start_rviz:=true`       | Open MoveIt RViz                                          |
| `setup_only:=false`      | Allow the exercise node to execute its reference workflow |
| `wait_timeout_sec:=30.0` | Set the endpoint wait timeout                             |

Pick-and-place and peg-in-hole move when `setup_only:=false`. Zero gravity also moves when `setup_only:=false`.

The host shortcut `./docker/container.sh sim` selects safe setup-only defaults. The `./docker/container.sh zero-g` shortcut runs the zero-gravity reference motion. The verifier selects motion defaults and checks the result.

## MoveIt Servo

Open three host terminals. Run `./docker/container.sh enter` in each terminal before using the commands below.

Start fake hardware in the first container shell:

```bash
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  start_rviz:=false use_fake_hardware:=true init_position:=false
```

Start MoveIt and Servo in the second container shell:

```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
  use_sim:=false start_rviz:=false
```

Run the example in the third container shell:

```bash
ros2 run open_manipulator_demos ex_servo
```

The example jogs `joint1` and verifies measured displacement. Change `joint_name` and `joint_speed` to select another arm joint or direction.

The 4-DOF arm does not support full Cartesian Servo commands. Use `ex_pose_goal` for Cartesian planning.

## Real hardware

Physical runs need USB access, clear workspace, conservative limits, and a tested stop procedure. Follow the guarded real-robot workflow in the [student guide](../README.md#use-the-real-robot).

Real hardware is outside the automated test environment. A passing fake-hardware or Gazebo check does not validate physical motion.

## Known warnings

MoveIt may report no 3D sensor plugin for Octomap updates. The course setup has no depth-sensor plugin, but robot-model and exercise-object collision checks still run.

The visual-only `end_effector_link` has no collision geometry. It marks the tool target frame; the physical gripper links contain the geometry.

Gazebo reports that its 5 ms physics step is faster than the 10 ms controller period. The simulation intentionally performs two physics steps per controller update.

Jazzy deprecates the compatible `GripperCommand` controller in favor of a controller with another action type. The demos retain the compatible controller used by `pymoveit2`.

On `Ctrl+C`, controller manager may print a `pal_statistics` error containing `context cannot be slept with because it's invalid`. It occurs after clean shutdown and does not mean the demo failed.

The [student guide](../README.md#expected-warnings) lists the remaining Gazebo, RViz, and graphics warnings seen in the supported Docker workflows.
