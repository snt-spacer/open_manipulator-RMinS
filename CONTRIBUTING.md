# Contributing

Keep changes focused, test them in the supported Docker environment, and update the relevant README when behavior or commands change.

## Prepare the workspace

Follow the [student setup guide](README.md#before-you-start), then build and start the current checkout:

```bash
./docker/container.sh start
```

Open a container shell:

```bash
./docker/container.sh enter
```

The shell opens in `/root/ros2_ws` with all ROS workspaces sourced.

After changing source, rebuild the workspace before testing:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For Dockerfile, Compose, package metadata, or dependency changes, leave the container shell and run `./docker/container.sh start` on the host. This rebuilds the image and recreates the container when needed.

## Validate a change

Run the ROS package tests inside the container:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For changes to demos, launch files, controllers, MoveIt, Gazebo, or Docker, also run the measured workflow verifier from the host:

```bash
./docker/container.sh verify-demos
```

Before submitting, check the patch from the host:

```bash
git diff --check
git status --short
```

Do not commit generated `build`, `install`, or `log` directories.

## Write documentation

Put student setup and task workflows in the root `README.md`. Put package-specific commands, parameters, and APIs in that package's README.

Commands should state whether they run on the host or in the container. Multi-terminal workflows should identify what stays running in each terminal.

Document expected output, nonzero failure behavior, units, coordinate frames, and known warnings when they affect how a student judges success.

## Sign off commits

Contributions use the [Developer Certificate of Origin](https://developercertificate.org/). Add a `Signed-off-by` line with Git's `-s` option:

```bash
git commit -s
```

By signing off, you certify that you have the right to submit the contribution under the repository's Apache 2.0 license.

Unless you state otherwise, an intentional contribution is submitted under Apache 2.0 without extra terms. A separate written license agreement still takes precedence when one applies.
