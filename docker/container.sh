#!/usr/bin/env bash

set -euo pipefail

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="open_manipulator"
CONTAINER_SOURCE_DIR="/root/ros2_ws/src/open_manipulator"
XAUTH_FILE="${OPEN_MANIPULATOR_XAUTH:-${TMPDIR:-/tmp}/open_manipulator_${UID}.xauth}"

is_container_running() {
    docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' \
        | grep -qx "${CONTAINER_NAME}"
}

is_container_current() {
    local container_image
    local current_image
    local container_compose_file
    local requested_compose_file

    container_image=$(docker inspect --format '{{.Image}}' "$CONTAINER_NAME") \
        || return 1
    current_image=$(docker image inspect --format '{{.Id}}' \
        robotis/open-manipulator:latest) || return 1
    container_compose_file=$(docker inspect --format \
        '{{ index .Config.Labels "com.docker.compose.project.config_files" }}' \
        "$CONTAINER_NAME") || return 1
    requested_compose_file=$(get_compose_file)

    [ "$container_image" = "$current_image" ] \
        && [ "$container_compose_file" = "$requested_compose_file" ]
}

detect_nvidia_gpu() {
    case "${OPEN_MANIPULATOR_GPU:-auto}" in
        general)
            echo "Using general graphics configuration by request." >&2
            return 1
            ;;
        nvidia)
            echo "Using NVIDIA graphics configuration by request." >&2
            return 0
            ;;
        auto)
            ;;
        *)
            echo "Error: OPEN_MANIPULATOR_GPU must be auto, general, or nvidia." >&2
            exit 2
            ;;
    esac

    if ! command -v nvidia-smi > /dev/null 2>&1 \
        || ! nvidia-smi > /dev/null 2>&1; then
        echo "No working NVIDIA GPU detected. Using general graphics configuration." >&2
        return 1
    fi

    if docker info --format '{{json .Runtimes}}' 2> /dev/null | grep -q '"nvidia"' \
        || [ -r /etc/cdi/nvidia.yaml ] \
        || [ -r /var/run/cdi/nvidia.yaml ]; then
        echo "NVIDIA GPU and Docker support detected." >&2
        return 0
    fi

    echo "NVIDIA GPU found, but Docker NVIDIA support is unavailable. Using general graphics configuration." >&2
    return 1
}
get_compose_file() {
    if detect_nvidia_gpu; then
        echo "${SCRIPT_DIR}/docker-compose.yml"
    else
        echo "${SCRIPT_DIR}/docker-compose-general.yml"
    fi
}

prepare_gui_environment() {
    if [ -e "$XAUTH_FILE" ] && [ ! -f "$XAUTH_FILE" ]; then
        echo "Error: Xauthority path is not a file: $XAUTH_FILE" >&2
        return 1
    fi

    touch "$XAUTH_FILE"
    chmod 600 "$XAUTH_FILE"
    export OPEN_MANIPULATOR_XAUTH="$XAUTH_FILE"

    if [ -z "${DISPLAY:-}" ]; then
        return 0
    fi

    if command -v xauth > /dev/null 2>&1; then
        local entries
        entries=$(xauth nlist "$DISPLAY" 2> /dev/null || true)
        if [ -n "$entries" ]; then
            printf '%s\n' "$entries" \
                | sed -e 's/^..../ffff/' \
                | xauth -f "$XAUTH_FILE" nmerge -
            return 0
        fi
    fi

    if command -v xhost > /dev/null 2>&1 \
        && xhost +SI:localuser:root > /dev/null 2>&1; then
        return 0
    fi

    echo "Warning: xauth and xhost could not grant container GUI access." >&2
}

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  build                   Build the container image"
    echo "  start                   Build if needed, then start the container"
    echo "  enter                   Start if needed, then enter the container"
    echo "  stop                    Stop the container"
    echo "  visual                  Launch fake hardware plus MoveIt RViz"
    echo "  sim                     Launch tabletop sim practice"
    echo "  zero-g                  Launch zero-g practice motion in the container"
    echo "  verify-demos            Verify direct and Gazebo motion workflows"
    echo "  real                    Show guarded real-robot launch path"
    echo ""
    echo "Examples:"
    echo "  $0 start                First setup or later startup"
    echo "  $0 enter                Start if needed, then enter the container"
    echo "  $0 stop                 Stop the container"
    echo "  $0 visual               Launch fake hardware plus MoveIt RViz"
    echo "  $0 sim                  Launch tabletop sim practice"
    echo "  $0 zero-g               Launch zero-g practice motion"
    echo "  $0 verify-demos         Verify direct and Gazebo motion workflows"
    echo "  $0 real                 Print guarded real-robot launch path"
}

build_container() {
    COMPOSE_FILE=$(get_compose_file)
    echo "Using compose file: $(basename "$COMPOSE_FILE")"
    docker compose -f "$COMPOSE_FILE" build
}

configure_host_udev_rules() {
    if ! command -v sudo > /dev/null 2>&1; then
        echo "Skipping host udev rules: sudo is unavailable."
        return 0
    fi
    if ! sudo -n true 2> /dev/null; then
        echo "Skipping host udev rules: sudo needs a password in this shell."
        echo "For real hardware, add the host user to dialout as described in README.md."
        return 0
    fi

    # Copy udev rule for FTDI (U2D2)
    echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' \
        | sudo tee /etc/udev/rules.d/99-u2d2.rules > /dev/null

    echo "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}

# Function to start the container
start_container() {
    prepare_gui_environment

    if [ -n "${DISPLAY:-}" ]; then
        echo "Setting up X11 forwarding..."
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    echo "Starting Open Manipulator container..."
    configure_host_udev_rules

    COMPOSE_FILE=$(get_compose_file)

    docker compose -f "$COMPOSE_FILE" up -d # --build
    echo "Container ready. Source workspace is already built and sourced."
}

ensure_container_running() {
    if is_container_running && is_container_current; then
        return 0
    fi
    if is_container_running; then
        echo 'Recreating container from the current image and graphics configuration.'
    fi
    start_container
}

container_display_is_available() {
    [ -n "${DISPLAY:-}" ] || return 1

    docker exec \
        --env "DISPLAY=${DISPLAY}" \
        --env XAUTHORITY=/tmp/.docker.xauth \
        "$CONTAINER_NAME" \
        python3 "${CONTAINER_SOURCE_DIR}/docker/check_x11.py" \
        > /dev/null 2>&1
}

require_container_display() {
    if container_display_is_available; then
        return 0
    fi

    # A reused container may have stale Xauthority data. Granting the local
    # root user access repairs that case without restarting the container.
    if [ -n "${DISPLAY:-}" ] && command -v xhost > /dev/null 2>&1; then
        if xhost +SI:localuser:root > /dev/null 2>&1 \
            && container_display_is_available; then
            echo "Repaired container access to X display ${DISPLAY}."
            return 0
        fi
    fi

    echo "Error: The container cannot connect to X display '${DISPLAY:-<unset>}'." >&2
    echo "Check the host DISPLAY and X server, then rerun the command." >&2
    echo "For a headless simulation, set OPEN_MANIPULATOR_HEADLESS=true and OPEN_MANIPULATOR_START_RVIZ=false." >&2
    return 1
}

# Function to enter the container
enter_container() {
    prepare_gui_environment

    if [ -n "${DISPLAY:-}" ]; then
        echo "Setting up X11 forwarding..."
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    ensure_container_running
    COMPOSE_FILE=$(get_compose_file)
    trap "docker compose -f '$COMPOSE_FILE' down >/dev/null 2>&1 || true" EXIT INT TERM
    docker exec \
        --env "DISPLAY=${DISPLAY:-}" \
        --env XAUTHORITY=/tmp/.docker.xauth \
        -it "$CONTAINER_NAME" bash
}

# Function to stop the container
stop_container() {
    if ! is_container_running; then
        echo "Error: Container is not running"
        exit 1
    fi

    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        COMPOSE_FILE=$(get_compose_file)
        echo "Using compose file: $(basename "$COMPOSE_FILE")"
        docker compose -f "$COMPOSE_FILE" down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

cleanup_container_runtime() {
    if ! is_container_running; then
        return 0
    fi

    docker exec "$CONTAINER_NAME" bash -lc '
        terminate_by_name() {
            signal="$1"
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
                ruby \
                ros2; do
                pkill "$signal" -x "$comm" 2>/dev/null || true
            done
        }

        terminate_by_pattern() {
            signal="$1"
            for pattern in \
                "[r]os2 launch" \
                "[r]os2 run open_manipulator_demos" \
                "[r]os2cli.daemon.daemonize" \
                "[o]pen_manipulator_demos/lib/open_manipulator_demos" \
                "[j]oint_trajectory_executor"; do
                pkill "$signal" -f "$pattern" 2>/dev/null || true
            done
        }

        terminate_by_name -TERM
        terminate_by_pattern -TERM
        sleep 2
        terminate_by_name -KILL
        terminate_by_pattern -KILL
    ' >/dev/null 2>&1 || true
}

cleanup_container_runtime_and_down() {
    local compose_file="$1"

    cleanup_container_runtime
    docker compose -f "$compose_file" down >/dev/null 2>&1 || true
}

launch_zero_g_practice() {
    prepare_gui_environment
    ensure_container_running

    if [ -n "${DISPLAY:-}" ]; then
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-true}
        HEADLESS=${OPEN_MANIPULATOR_HEADLESS:-false}
    else
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-false}
        HEADLESS=${OPEN_MANIPULATOR_HEADLESS:-true}
    fi

    if [ "$START_RVIZ" = "true" ] || [ "$HEADLESS" != "true" ]; then
        require_container_display
    fi

    COMPOSE_FILE=$(get_compose_file)
    trap "cleanup_container_runtime_and_down '$COMPOSE_FILE'" EXIT INT TERM
    docker exec \
        --env "DISPLAY=${DISPLAY:-}" \
        --env XAUTHORITY=/tmp/.docker.xauth \
        "$CONTAINER_NAME" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /opt/open_manipulator_deps_ws/install/setup.bash
        source /root/ros2_ws/install/setup.bash
        ros2 launch open_manipulator_demos demo_zero_g_practice.launch.py \
            start_rviz:=${START_RVIZ} \
            headless:=${HEADLESS} \
            setup_only:=false
    "
}

launch_visual_practice() {
    prepare_gui_environment
    ensure_container_running

    if [ -n "${DISPLAY:-}" ]; then
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-true}
    else
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-false}
    fi
    INIT_POSITION=${OPEN_MANIPULATOR_INIT_POSITION:-true}

    if [ "$START_RVIZ" = "true" ]; then
        require_container_display
    fi

    COMPOSE_FILE=$(get_compose_file)
    trap "cleanup_container_runtime_and_down '$COMPOSE_FILE'" EXIT INT TERM
    docker exec \
        --env "DISPLAY=${DISPLAY:-}" \
        --env XAUTHORITY=/tmp/.docker.xauth \
        "$CONTAINER_NAME" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /opt/open_manipulator_deps_ws/install/setup.bash
        source /root/ros2_ws/install/setup.bash
        ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
            start_rviz:=false \
            use_fake_hardware:=true \
            init_position:=${INIT_POSITION} &
        sleep 6
        ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
            use_sim:=false \
            start_rviz:=${START_RVIZ}
    "
}

launch_sim_practice() {
    prepare_gui_environment
    ensure_container_running

    if [ -n "${DISPLAY:-}" ]; then
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-true}
        HEADLESS=${OPEN_MANIPULATOR_HEADLESS:-false}
    else
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-false}
        HEADLESS=${OPEN_MANIPULATOR_HEADLESS:-true}
    fi

    if [ "$START_RVIZ" = "true" ] || [ "$HEADLESS" != "true" ]; then
        require_container_display
    fi

    COMPOSE_FILE=$(get_compose_file)
    trap "cleanup_container_runtime_and_down '$COMPOSE_FILE'" EXIT INT TERM
    docker exec \
        --env "DISPLAY=${DISPLAY:-}" \
        --env XAUTHORITY=/tmp/.docker.xauth \
        "$CONTAINER_NAME" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /opt/open_manipulator_deps_ws/install/setup.bash
        source /root/ros2_ws/install/setup.bash
        ros2 launch open_manipulator_demos demo_pick_and_place.launch.py \
            start_rviz:=${START_RVIZ} \
            headless:=${HEADLESS} \
            setup_only:=true
    "
}

verify_demo_workflows() {
    ensure_container_running

    echo 'Preparing an isolated fake-hardware verification run...'
    cleanup_container_runtime
    trap cleanup_container_runtime EXIT INT TERM

    docker exec "$CONTAINER_NAME" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /opt/open_manipulator_deps_ws/install/setup.bash
        source /root/ros2_ws/install/setup.bash
        ${CONTAINER_SOURCE_DIR}/docker/verify_demos.sh
    "
}

launch_real_robot() {
    ensure_container_running

    PORT_NAME=${OPEN_MANIPULATOR_PORT_NAME:-/dev/ttyUSB0}
    if [ -n "${DISPLAY:-}" ]; then
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-true}
    else
        START_RVIZ=${OPEN_MANIPULATOR_START_RVIZ:-false}
    fi
    INIT_POSITION=${OPEN_MANIPULATOR_INIT_POSITION:-true}

    if [ "$START_RVIZ" = "true" ]; then
        require_container_display
    fi

    COMPOSE_FILE=$(get_compose_file)

    lower_arm_to_rest() {
        if ! is_container_running; then
            return 0
        fi

        docker exec \
            --env "DISPLAY=${DISPLAY:-}" \
            --env XAUTHORITY=/tmp/.docker.xauth \
            "$CONTAINER_NAME" bash -lc '
            set -e
            source /opt/ros/jazzy/setup.bash
            source /opt/open_manipulator_deps_ws/install/setup.bash
            source /root/ros2_ws/install/setup.bash
            timeout -k 1s 8s python3 /root/ros2_ws/src/open_manipulator/open_manipulator_bringup/open_manipulator_bringup/shutdown_lander.py
        ' >/dev/null 2>&1 || true
    }

    trap "lower_arm_to_rest; cleanup_container_runtime_and_down '$COMPOSE_FILE'" EXIT INT TERM
    docker exec "$CONTAINER_NAME" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /opt/open_manipulator_deps_ws/install/setup.bash
        source /root/ros2_ws/install/setup.bash
        ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
            port_name:=${PORT_NAME} \
            start_rviz:=false \
            init_position:=${INIT_POSITION} &
        sleep 6
        ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
            use_sim:=false \
            start_rviz:=${START_RVIZ}
    "
}

# Main command handling
case "${1:-help}" in
    "help")
        show_help
        ;;
    "build")
        build_container
        ;;
    "start")
        start_container
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    "visual")
        launch_visual_practice
        ;;
    "sim")
        launch_sim_practice
        ;;
    "zero-g")
        launch_zero_g_practice
        ;;
    "verify-demos")
        verify_demo_workflows
        ;;
    "real")
        launch_real_robot
        ;;
    *)
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac
