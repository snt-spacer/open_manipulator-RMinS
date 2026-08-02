#!/usr/bin/env bash

set -euo pipefail

BRINGUP_LOG=/tmp/open_manipulator_verify_bringup.log
MOVEIT_LOG=/tmp/open_manipulator_verify_moveit.log
GAZEBO_LOG=/tmp/open_manipulator_verify_gazebo.log
BRINGUP_PID=''
MOVEIT_PID=''
ACTIVE_PID=''

show_failure_logs() {
    for label_and_path in \
        "Bringup:$BRINGUP_LOG" \
        "MoveIt:$MOVEIT_LOG" \
        "Gazebo:$GAZEBO_LOG"; do
        local label=${label_and_path%%:*}
        local path=${label_and_path#*:}
        echo "$label log:" >&2
        tail -n 100 "$path" >&2 || true
    done
}

stop_process() {
    local pid=$1
    if [ -z "$pid" ]; then
        return 0
    fi

    kill -TERM "$pid" 2> /dev/null || true
    for _ in $(seq 1 50); do
        if ! kill -0 "$pid" 2> /dev/null; then
            wait "$pid" 2> /dev/null || true
            return 0
        fi
        sleep 0.1
    done
    kill -KILL "$pid" 2> /dev/null || true
    wait "$pid" 2> /dev/null || true
}

cleanup_orphans() {
    for comm in \
        move_group \
        ros2_control_no \
        spawner \
        robot_state_pub \
        joint_state_pub \
        parameter_bridg \
        controller_mana \
        servo_node \
        rviz2 \
        gz \
        ruby; do
        pkill -TERM -x "$comm" 2> /dev/null || true
    done
    pkill -TERM -f '[r]os2cli.daemon.daemonize' 2> /dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM

    stop_process "$ACTIVE_PID"
    stop_process "$MOVEIT_PID"
    stop_process "$BRINGUP_PID"
    cleanup_orphans

    if [ "$status" -ne 0 ]; then
        show_failure_logs
    fi
    exit "$status"
}

wait_for_endpoint() {
    local kind=$1
    local endpoint=$2
    local attempts=120

    while [ "$attempts" -gt 0 ]; do
        if ros2 "$kind" list 2> /dev/null | grep -Fxq "$endpoint"; then
            return 0
        fi
        sleep 0.5
        attempts=$((attempts - 1))
    done

    echo "Timed out waiting for ROS $kind endpoint: $endpoint" >&2
    return 1
}

wait_for_log() {
    local pid=$1
    local log_path=$2
    local success_text=$3
    local attempts=180

    while [ "$attempts" -gt 0 ]; do
        if grep -Fq "$success_text" "$log_path"; then
            return 0
        fi
        if ! kill -0 "$pid" 2> /dev/null; then
            wait "$pid"
            echo "Process exited before logging: $success_text" >&2
            return 1
        fi
        sleep 0.5
        attempts=$((attempts - 1))
    done

    echo "Timed out waiting for log message: $success_text" >&2
    return 1
}

run_checked_example() {
    local output

    if ! output=$("$@" 2>&1); then
        printf '%s\n' "$output" >&2
        return 1
    fi
    printf '%s\n' "$output"

    if grep -Fq 'exception was never retrieved' <<< "$output"; then
        echo 'Example exited with an unhandled asynchronous exception.' >&2
        return 1
    fi
}

run_expect_failure() {
    local expected_text=$1
    shift
    local output

    if output=$("$@" 2>&1); then
        printf '%s\n' "$output" >&2
        echo "Command unexpectedly succeeded; expected: $expected_text" >&2
        return 1
    fi
    if ! grep -Fq "$expected_text" <<< "$output"; then
        printf '%s\n' "$output" >&2
        echo "Failure did not contain: $expected_text" >&2
        return 1
    fi
    grep -F "$expected_text" <<< "$output" | tail -n 1
}

run_primitive_variant() {
    local shape=$1
    local object_id=$2
    local dimensions=$3

    run_checked_example ros2 run open_manipulator_demos ex_collision_primitive \
        --ros-args \
        -p shape:="$shape" \
        -p object_id:="$object_id" \
        -p dimensions:="$dimensions"
    run_checked_example ros2 run open_manipulator_demos ex_collision_primitive \
        --ros-args \
        -p shape:="$shape" \
        -p action:='remove' \
        -p object_id:="$object_id" \
        -p dimensions:="$dimensions"
}

run_gazebo_workflow() {
    local launch_file=$1
    local success_text=$2
    shift 2

    : > "$GAZEBO_LOG"
    echo "Running Gazebo workflow: $launch_file"
    ros2 launch open_manipulator_demos "$launch_file" \
        start_rviz:=false \
        headless:=true \
        "$@" \
        > "$GAZEBO_LOG" 2>&1 &
    ACTIVE_PID=$!

    wait_for_log "$ACTIVE_PID" "$GAZEBO_LOG" "$success_text"
    grep -F "$success_text" "$GAZEBO_LOG" | tail -n 1
    stop_process "$ACTIVE_PID"
    ACTIVE_PID=''
    cleanup_orphans
    sleep 2
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

: > "$BRINGUP_LOG"
: > "$MOVEIT_LOG"
: > "$GAZEBO_LOG"

echo 'Running direct examples against fake hardware...'
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
    start_rviz:=false \
    use_fake_hardware:=true \
    init_position:=false \
    > "$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!

wait_for_endpoint action /gripper_controller/gripper_cmd

ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
    use_sim:=false \
    start_rviz:=false \
    > "$MOVEIT_LOG" 2>&1 &
MOVEIT_PID=$!

wait_for_endpoint action /move_action
wait_for_endpoint service /get_planning_scene
wait_for_endpoint service /servo_node/pause_servo
wait_for_endpoint service /servo_node/switch_command_type

run_checked_example ros2 run open_manipulator_demos template
run_checked_example ros2 run open_manipulator_demos ex_joint_goal \
    --ros-args \
    -p joint_positions:='[0.15, -0.40, 0.65, 0.10]' \
    -p max_velocity:=0.15 \
    -p max_acceleration:=0.15
run_checked_example ros2 run open_manipulator_demos ex_joint_goal \
    --ros-args \
    -p joint_positions:='[-0.15, -0.55, 0.45, -0.10]' \
    -p max_velocity:=0.20 \
    -p max_acceleration:=0.20
run_checked_example ros2 run open_manipulator_demos ex_pose_goal \
    --ros-args \
    -p planning_attempts:=5 \
    -p allowed_planning_time:=4.0
run_checked_example ros2 run open_manipulator_demos ex_pose_goal \
    --ros-args \
    -p position:='[0.006, 0.0, 0.0]' \
    -p cartesian:=true \
    -p relative_cartesian:=true \
    -p position_tolerance:=0.003 \
    -p allowed_planning_time:=4.0 \
    -p max_velocity:=0.10 \
    -p max_acceleration:=0.10
run_checked_example ros2 run open_manipulator_demos ex_gripper \
    --ros-args -p action:='close'
run_checked_example ros2 run open_manipulator_demos ex_gripper \
    --ros-args -p action:='toggle'
run_checked_example ros2 run open_manipulator_demos ex_gripper \
    --ros-args -p action:='toggle'
run_checked_example ros2 run open_manipulator_demos ex_gripper \
    --ros-args -p action:='open'
run_checked_example ros2 run open_manipulator_demos ex_utils_fixed
run_checked_example ros2 run open_manipulator_demos ex_utils_fixed
run_checked_example ros2 run open_manipulator_demos ex_servo \
    --ros-args \
    -p joint_name:=joint1 \
    -p joint_speed:=0.08 \
    -p duration_sec:=1.5
run_checked_example ros2 run open_manipulator_demos ex_servo \
    --ros-args \
    -p joint_name:=joint2 \
    -p joint_speed:=-0.06 \
    -p frequency_hz:=25.0 \
    -p duration_sec:=1.5
run_primitive_variant box verify_box '[0.04, 0.03, 0.02]'
run_primitive_variant sphere verify_sphere '[0.025]'
run_primitive_variant cylinder verify_cylinder '[0.08, 0.02]'
run_primitive_variant cone verify_cone '[0.07, 0.025]'
run_checked_example ros2 run open_manipulator_demos ex_collision_mesh \
    --ros-args \
    -p object_id:=verify_mesh \
    -p position:='[0.28, 0.18, 0.12]' \
    -p quat_xyzw:='[0.0, 0.0, 0.70710678, 0.70710678]' \
    -p scale:=0.025
run_checked_example ros2 run open_manipulator_demos ex_collision_mesh \
    --ros-args \
    -p action:='remove' \
    -p object_id:=verify_mesh

run_checked_example ros2 run open_manipulator_demos ex_joint_goal
run_checked_example ros2 run open_manipulator_demos ex_pose_goal
run_checked_example ros2 run open_manipulator_demos ex_gripper
run_checked_example ros2 run open_manipulator_demos ex_servo
run_checked_example ros2 run open_manipulator_demos ex_collision_primitive
run_checked_example ros2 run open_manipulator_demos ex_collision_mesh

echo 'Checking rejected parameter variants...'
run_expect_failure 'wait_timeout_sec must be positive' \
    ros2 run open_manipulator_demos ex_joint_goal \
    --ros-args -p wait_timeout_sec:=0.0
run_expect_failure 'quat_xyzw must be normalized' \
    ros2 run open_manipulator_demos ex_pose_goal \
    --ros-args -p quat_xyzw:='[0.0, 0.0, 0.0, 0.0]'
run_expect_failure 'action must be one of: open, close, toggle' \
    ros2 run open_manipulator_demos ex_gripper \
    --ros-args -p action:=invalid
run_expect_failure 'Cartesian Servo commands are unsupported' \
    ros2 run open_manipulator_demos ex_servo \
    --ros-args -p command_mode:=twist
run_expect_failure 'box dimensions values must be positive' \
    ros2 run open_manipulator_demos ex_collision_primitive \
    --ros-args -p dimensions:='[-0.04, 0.04, 0.04]'
run_expect_failure 'scale must be positive' \
    ros2 run open_manipulator_demos ex_collision_mesh \
    --ros-args -p scale:=-0.04
run_expect_failure 'table_size values must be positive' \
    ros2 run open_manipulator_demos demo_pick_and_place \
    --ros-args -p table_size:='[0.30, -0.24, 0.02]'
run_expect_failure 'module_size values must be positive' \
    ros2 run open_manipulator_demos demo_peg_in_hole \
    --ros-args -p module_size:='[0.16, 0.11, 0.0]'
run_expect_failure 'motion_mode must be "joint"' \
    ros2 run open_manipulator_demos demo_zero_g_practice \
    --ros-args -p motion_mode:=pose

stop_process "$MOVEIT_PID"
MOVEIT_PID=''
stop_process "$BRINGUP_PID"
BRINGUP_PID=''
cleanup_orphans
sleep 2

echo 'Checking missing-stack guidance...'
run_expect_failure "Keep './docker/container.sh visual' running" \
    ros2 run open_manipulator_demos template \
    --ros-args -p wait_timeout_sec:=0.25
run_expect_failure "Keep './docker/container.sh visual' running" \
    ros2 run open_manipulator_demos ex_joint_goal \
    --ros-args -p wait_timeout_sec:=0.25
run_expect_failure "Keep './docker/container.sh visual' running" \
    ros2 run open_manipulator_demos ex_pose_goal \
    --ros-args -p wait_timeout_sec:=0.25
run_expect_failure "Keep './docker/container.sh visual' running" \
    ros2 run open_manipulator_demos ex_gripper \
    --ros-args -p wait_timeout_sec:=0.25
run_expect_failure "Keep './docker/container.sh visual' running" \
    ros2 run open_manipulator_demos ex_collision_primitive \
    --ros-args -p wait_timeout_sec:=0.25
run_expect_failure "Keep './docker/container.sh visual' running" \
    ros2 run open_manipulator_demos ex_collision_mesh \
    --ros-args -p wait_timeout_sec:=0.25
run_expect_failure 'start MoveIt in another container shell' \
    ros2 run open_manipulator_demos ex_servo \
    --ros-args -p wait_timeout_sec:=0.25

run_gazebo_workflow \
    demo_pick_and_place.launch.py \
    'pick/place setup complete; no motion commanded' \
    setup_only:=true
run_gazebo_workflow \
    demo_pick_and_place.launch.py \
    'pick/place reference motion complete:' \
    setup_only:=false
run_gazebo_workflow \
    demo_peg_in_hole.launch.py \
    'peg/hole setup complete; no motion commanded' \
    setup_only:=true
run_gazebo_workflow \
    demo_peg_in_hole.launch.py \
    'peg/hole reference motion complete:' \
    setup_only:=false
run_gazebo_workflow \
    demo_zero_g_practice.launch.py \
    'zero-g practice setup complete; no motion commanded' \
    setup_only:=true
run_gazebo_workflow \
    demo_zero_g_practice.launch.py \
    'zero-g retreat from floating sample measured at target:' \
    setup_only:=false

echo 'Direct examples and all Gazebo motion workflows passed.'

