# Robotics and Manipulation II (2026)

This repository contains the OpenMANIPULATOR-X software and course exercises. The supported student environment is ROS 2 Jazzy in Docker. You do not need to install ROS on the host.

## Before you start

The Docker setup targets a Linux host. It uses host networking, Linux devices under `/dev`, and X11 for Gazebo and RViz.

You need:

- Docker Engine with the Docker Compose plugin
- Git
- an X11 display for graphical use
- an NVIDIA container runtime only when you want NVIDIA acceleration

Check the required commands:

```bash
docker --version
docker compose version
git --version
```

Allow your user to run Docker without `sudo` and to the serial-device group:

```bash
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER
```

Log out and back in after changing the group. Confirm that `docker ps` works before continuing.

## Quick start

Clone the repository and start the fake-hardware exercise:

```bash
git clone https://github.com/snt-spacer/open_manipulator-RMinS.git
cd open_manipulator-RMinS
./docker/container.sh visual
```

The first run builds the image and can take several minutes. Later runs use Docker's build cache.

RViz should open with the OpenMANIPULATOR-X model and MoveIt controls. Keep the command running while you work. Press `Ctrl+C` when you want to stop the ROS stack.

The workspace is already built and sourced in the image. You do not need a separate `colcon build` or `source` command for the first run.

## Choose a workflow

Run all commands in this table from the repository root on the host.

| Command                        | What it does                                             |
| ------------------------------ | -------------------------------------------------------- |
| `./docker/container.sh visual` | Starts fake (visual-only) hardware, MoveIt, and RViz     |
| `./docker/container.sh sim`    | Starts the simulated tabletop Gazebo scene and RViz      |
| `./docker/container.sh zero-g` | Starts the simulated zero-gravity Gazebo motion and RViz |
| `./docker/container.sh real`   | Starts the real robot bringup, MoveIt, and RViz          |
| `./docker/container.sh help`   | Shows the complete shortcut list                         |

The `visual`, `sim`, `zero-g`, and `real` commands remain attached to the current terminal. The container is removed automatically when you press `Ctrl+C`, so you do not need to stop it manually.

> The `real` workflow starts the bringup stack first, then launches MoveIt with RViz so the planning scene is available alongside the controller interfaces. When you press `Ctrl+C`, the wrapper first sends the arm back to the configured home posture slowly, then tears the stack down and removes the container.

## Work in two terminals

Start a ROS stack in the first host terminal:

```bash
./docker/container.sh visual
```

Keep it running. In a second host terminal, change to the same repository and enter the container:

```bash
cd open_manipulator-RMinS
./docker/container.sh enter
```

The shell opens with ROS 2, the dependency workspace, and the course workspace sourced. Commands such as `ros2 node list` are ready to use.

The container keeps a persistent bash history in a named Docker volume, and the repository is bind-mounted into the container, so edits survive when the container is torn down.
The `visual` and `real` workflows also move the arm to its configured home position before MoveIt starts, which avoids planning from the boot-time self-collision pose. Set `OPEN_MANIPULATOR_INIT_POSITION=false` if you need to keep the raw boot state.

## Run direct MoveIt examples

Start fake hardware and MoveIt in the first host terminal:

```bash
./docker/container.sh visual
```

Enter the container from a second host terminal:

```bash
./docker/container.sh enter
```

Confirm that MoveIt, the gripper controller, and the planning scene are available:

```bash
ros2 action list
ros2 service list
```

The action list must contain `/move_action` and `/gripper_controller/gripper_cmd`. The service list must contain `/get_planning_scene` and `/apply_planning_scene`.

Run any of these examples from the container shell:

```bash
ros2 run open_manipulator_demos ex_joint_goal
ros2 run open_manipulator_demos ex_pose_goal
ros2 run open_manipulator_demos ex_gripper --ros-args -p action:="toggle" # "toggle", "open", or "close"
ros2 run open_manipulator_demos ex_servo
ros2 run open_manipulator_demos ex_collision_primitive
ros2 run open_manipulator_demos ex_collision_mesh
```

The motion examples return a nonzero status when planning, execution, or measured-state validation fails. Scene examples also wait for MoveIt to confirm each object update. Servo has a separate startup procedure below.

### Pose goal

`ex_pose_goal` accepts a position in metres and a normalized quaternion in `xyzw` order. Its target frame is `world`.

```bash
ros2 run open_manipulator_demos ex_pose_goal --ros-args \
  -p position:="[0.0, -0.042, 0.134]" \
  -p quat_xyzw:="[0.175, 0.175, -0.685, 0.685]"
```

The joint and pose examples use the `arm` planning group and `end_effector_link` as the tool target.

The OpenMANIPULATOR-X MoveIt configuration uses position-only IK because the arm has four degrees of freedom. The example verifies the measured tool position after motion. It reports the measured quaternion, but does not claim that the requested orientation was reached.

`cartesian:=true` requests a straight tool path from the current pose. Use it for a short nearby waypoint. A distant Cartesian target can have only a partial path even when the same target is reachable with normal planning.

For a short offset from the measured tool pose, set `relative_cartesian:=true` with `cartesian:=true`. In this mode, `position` is an `[x, y, z]` offset in metres. The request uses the current tool orientation, but position-only IK still verifies only the final position:

```bash
ros2 run open_manipulator_demos ex_pose_goal --ros-args \
  -p cartesian:=true \
  -p relative_cartesian:=true \
  -p position:='[0.01, 0.0, 0.0]' \
  -p position_tolerance:=0.003
```

## Run Gazebo exercises

The shortcuts start the container-side workflows for the scenes. `sim` stays in default Earth gravity, while `zero-g` runs the zero-gravity reference motion:

```bash
./docker/container.sh sim
./docker/container.sh zero-g
```

Run one launch command at a time from the container shell:

```bash
ros2 launch open_manipulator_demos demo_pick_and_place.launch.py \
  start_rviz:=false headless:=true

ros2 launch open_manipulator_demos demo_peg_in_hole.launch.py \
  start_rviz:=false headless:=true

ros2 launch open_manipulator_demos demo_zero_g_practice.launch.py \
  start_rviz:=false headless:=true
```

## Extras (optional)

### Update or rebuild the workspace

The checkout is mounted at `/root/ros2_ws/src/open_manipulator` inside the container. Edits under that directory are edits to the host checkout and remain after the container stops.

Files created elsewhere in the container are temporary. Move any work that must persist into the mounted repository.

Stop an active exercise first. After pulling changes or editing package metadata, CMake files, launch files, configuration, or compiled code, rebuild and recreate the container:

```bash
./docker/container.sh start
```

The command runs `docker compose up -d --build`. It reuses cached image layers when their inputs have not changed.

### Verify the installation

Stop any interactive exercise, then run the complete automated check from one host terminal:

```bash
./docker/container.sh verify-demos
```

The verifier owns the ROS processes in the container. It may stop another course launch that is still running there.

The check covers repeated arm goals, a short Cartesian waypoint, gripper state changes, two Servo joint directions, every supported collision shape, custom mesh parameters, setup-only launches, and all three Gazebo motion workflows. It also checks common invalid inputs and the guidance shown when a required ROS endpoint is missing. Motion checks use measured robot state instead of trusting action results alone.

A successful run ends with:

```text
Direct examples and all Gazebo motion workflows passed.
```

Any nonzero exit status means at least one check failed. Read the first reported failure before rerunning the entire suite.

### Advanced options

The container uses ROS domain ID 30 by default. To use another domain, set it when creating the container:

```bash
ROS_DOMAIN_ID=42 ./docker/container.sh visual
```

An existing container keeps the domain selected at creation. Stop and recreate it to change domains. Container shells opened with `enter` inherit its value. Host-side ROS processes must use the same value.

When `DISPLAY` is set, `visual`, `sim`, `zero-g`, and `real` start their GUI stack with MoveIt and RViz. The wrapper checks X11 access inside the container before starting Qt.

To use an existing X server on display `:99`:

```bash
DISPLAY=:99 ./docker/container.sh sim
```

To run without a display:

```bash
OPEN_MANIPULATOR_HEADLESS=true \
OPEN_MANIPULATOR_START_RVIZ=false \
./docker/container.sh sim
```

The wrapper selects NVIDIA support only when `nvidia-smi` works and Docker exposes an NVIDIA runtime or CDI configuration. Otherwise, it uses the general graphics configuration.

You can override the graphics choice:

```bash
OPEN_MANIPULATOR_GPU=general ./docker/container.sh sim
OPEN_MANIPULATOR_GPU=nvidia ./docker/container.sh sim
```

Use `general` when automatic NVIDIA detection selects a runtime that is not usable on the host.

## Troubleshooting

### Docker permission denied

Run `groups` and confirm that it contains `docker`. If you just added the group, log out and back in. Then check `docker ps` before rerunning the wrapper.

### Gazebo or RViz does not open

Run `echo $DISPLAY` and `xdpyinfo` on the host. The wrapper stops before launching Qt when the container cannot open the display.

See [Advanced options](#advanced-options) for headless and alternate-display overrides.

### Source changes are missing

Run `./docker/container.sh start` from the repository root. This rebuilds the image and recreates the container while preserving files in the host checkout.

### ROS endpoints are unavailable

Keep the launch command running in the first terminal. Enter the same container in the second terminal, and confirm that both host shells use the same `ROS_DOMAIN_ID`. See [Advanced options](#advanced-options) if you need to change it.

Use `ros2 action list` and `ros2 service list` to identify the missing endpoint. A direct example cannot create MoveIt or a controller by itself.

For `ex_servo`, use the two launch commands in [MoveIt Servo](#moveit-servo). The `visual` shortcut does not start the Servo node.

### MoveIt cannot plan

Use `use_sim:=true` with Gazebo. Use `use_sim:=false` with fake or real hardware. Check that the target is reachable and that its quaternion is normalized.

### USB permission denied

Run `groups` and confirm that it contains `dialout`. Reconnect the U2D2, then check that `OPEN_MANIPULATOR_PORT_NAME` points to the correct `/dev/ttyUSB*` device.

### Duplicate nodes or ports already in use

Stop old launch terminals with `Ctrl+C`. If processes remain, run `./docker/container.sh stop`, confirm removal, and start the required workflow again.

### Expected warnings

These messages describe known limits of the course setup:

- MoveIt reports no 3D sensor plugin for Octomap updates because the model has no depth-sensor plugin.
- `end_effector_link` has no collision geometry because it is a target-frame marker. The gripper links contain the tool geometry.
- Gazebo reports a 5 ms physics step and a 10 ms controller period. The controller intentionally updates every two physics steps.
- Gazebo does not support the gripper mimic constraint used by the URDF. The ros2_control gripper controller still drives the exercise interface.
- Jazzy deprecates the compatible `GripperCommand` controller. The replacement uses another action type, so these examples retain the compatible controller.
- RViz may report that `/recognize_objects` is unavailable. The exercises do not use object recognition.
- MoveIt RViz may report an `InteractiveMarkerDisplay` plugin factory collision and duplicate `rviz2_moveit` publishers. One responsive RViz window with `MotionPlanning` status `Ok` is healthy.
- General graphics mode may print `libEGL` or `failed to create dri2 screen` warnings on an NVIDIA host. They are harmless only when Gazebo opens and renders the scene.

On `Ctrl+C`, controller manager may log a `pal_statistics` error containing `context cannot be slept with because it's invalid`. It appears after clean controller and hardware shutdown and does not indicate a failed exercise.

Unexpected display errors, process crashes, Python tracebacks, unavailable endpoints, failed motion validation, and nonzero verifier exits are not expected warnings.

## More documentation

- [Demo commands, parameters, and helper API](open_manipulator_demos/README.md)
- [Contribution workflow](CONTRIBUTING.md)
