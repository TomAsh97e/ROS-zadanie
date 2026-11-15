#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$REPO_ROOT/install/setup.bash"

TIMESTAMP=$(date +'%d-%m-%y_%H:%M:%S')
CAPTURE_DIR="$REPO_ROOT/performance_analysis/capture_$TIMESTAMP"
MEM_DIR="$CAPTURE_DIR/mem"
CPU_DIR="$CAPTURE_DIR/cpu"

mkdir -p "$MEM_DIR"
mkdir -p "$CPU_DIR"


EXPECTED_NODES=(
    "ball_detector"
    "ball_grabber"
    "gimbal_controller"
    "gimbal_tracker"
    "hover_controller"
    "mission_behavior_tree"
    "pilot_node"
    "raycasted_approach_coordinator_node"
)

declare -A NODE_EXECUTABLES=(
    ["ball_detector"]="ball_detector"
    ["ball_grabber"]="ball_grabber"
    ["gimbal_controller"]="gimbal_controller"
    ["gimbal_tracker"]="gimbal_tracker"
    ["hover_controller"]="hover_controller"
    ["mission_behavior_tree"]="behavior_tree"
    ["pilot_node"]="pilot"
    ["raycasted_approach_coordinator_node"]="raycasted_approach_coordinator_node"
)

node_is_running() {
    local node="$1"
    grep -Eq "(^|/)${node}$" <<< "$NODE_LIST"
}

node_pid() {
    local node="$1"
    local executable="${NODE_EXECUTABLES[$node]}"
    pgrep -n -f "(^|/)${executable}(\.py)?( |$)"
}

stop_capture_and_plot() {
    kill "$MEM_PID" "$CPU_PID" 2>/dev/null
    wait
    echo -e "Plotting results...\n"
    python3 "$SCRIPT_DIR/plot_performance.py" "$CAPTURE_DIR" \
        || echo -e "\nFailed to make plots automatically. Use 'plot_performance.py' to do this manually."
    exit 0
}

echo -e "Please run mission in another terminal!\n"

echo -e "Waiting for all expected nodes to be running...\n"

PIDS=()
while true; do
    NODE_LIST="$(ros2 node list 2>/dev/null)"
    PIDS=()
    MISSING=()

    for node in "${EXPECTED_NODES[@]}"; do
        pid=""
        if node_is_running "$node"; then
            pid="$(node_pid "$node")"
        fi

        if [ -n "$pid" ]; then
            PIDS+=("$pid")
        else
            MISSING+=("$node")
        fi
    done

    if [ ${#MISSING[@]} -eq 0 ]; then
        break
    fi

    echo "Still waiting for: ${MISSING[*]}"
    sleep 1
done

PID_LIST=$(IFS=,; echo "${PIDS[*]}")
echo -e "All nodes are running. PIDs: $PID_LIST\n"
echo -e "Logging to capture_$TIMESTAMP (Ctrl+C to stop)\n"

COMMON_PID_ARGS=(1 -h -H -I -p "$PID_LIST")
pidstat "${COMMON_PID_ARGS[@]}" -r > "$MEM_DIR/pidstat.log" &
MEM_PID=$!
pidstat "${COMMON_PID_ARGS[@]}" -u > "$CPU_DIR/pidstat.log" &
CPU_PID=$!

trap stop_capture_and_plot INT TERM

wait "$MEM_PID" "$CPU_PID"
