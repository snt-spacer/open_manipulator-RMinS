#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="open_manipulator"

detect_nvidia_gpu() {
    if command -v nvidia-smi &> /dev/null; then
        if nvidia-smi > /dev/null 2>&1; then
            echo "NVIDIA GPU detected" >&2 
            return 0
        fi
    fi
    echo "No NVIDIA GPU detected, proceeding with general configuration." >&2
    return 1
}
get_compose_file() {
    if detect_nvidia_gpu; then
        echo "${SCRIPT_DIR}/docker-compose.yml"
    else
        echo "${SCRIPT_DIR}/docker-compose-general.yml"
    fi
}

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  start                   Start the container"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 start                Start container"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop the container"
}

# Function to start the container
start_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    echo "Starting Open Manipulator container..."

    # Copy udev rule for FTDI (U2D2)
    echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-u2d2.rules > /dev/null

    # Reload udev rules
    echo "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    COMPOSE_FILE=$(get_compose_file)

    # Pull the latest images
    docker compose -f "$COMPOSE_FILE" pull

    # Run docker-compose
    docker compose -f "$COMPOSE_FILE" up -d
}

# Function to enter the container
enter_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    docker exec -it "$CONTAINER_NAME" bash
}

# Function to stop the container
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
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

# Main command handling
case "$1" in
    "help")
        show_help
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
    *)
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac