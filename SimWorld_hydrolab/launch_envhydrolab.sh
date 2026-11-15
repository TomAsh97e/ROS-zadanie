#!/bin/bash
set -e

# ==========================================
# Paths (override with environment variables)
#   PX4_ROOT        - PX4-Autopilot checkout        (default: ~/PX4-Autopilot)
#   PX4_WORLDS_DIR  - where the world is installed  (default: $PX4_ROOT/Tools/simulation/gz/worlds)
#   LAUNCH_MISSION  - 1: also start the mission, 0: simulation only (default: 1)
#   HEADLESS        - 1: run Gazebo without GUI (handled by PX4)
# ==========================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
export PX4_WORLDS_DIR="${PX4_WORLDS_DIR:-$PX4_ROOT/Tools/simulation/gz/worlds}"
LAUNCH_MISSION="${LAUNCH_MISSION:-1}"
export HYDROLAB_CONFIG_DIR="$REPO_ROOT/runtime_config/hydrolab_mission"

source "$REPO_ROOT/install/setup.bash"
echo -e "source $REPO_ROOT/install/setup.bash\n"

# ==========================================
# Preflight checks (before anything is started)
# ==========================================
fail() {
    echo "ERROR: $*" >&2
    exit 1
}

[ -d "$PX4_ROOT" ] || fail "PX4-Autopilot not found at '$PX4_ROOT'. Set PX4_ROOT=<path to PX4-Autopilot>."
[ -d "$PX4_WORLDS_DIR" ] || fail "PX4 worlds directory not found at '$PX4_WORLDS_DIR'. Set PX4_WORLDS_DIR."
command -v MicroXRCEAgent >/dev/null || fail "MicroXRCEAgent not found (see README: Micro XRCE-DDS Agent)."

if [ "$LAUNCH_MISSION" = "1" ]; then
    [ -f /opt/ros/jazzy/setup.bash ] || fail "ROS 2 Jazzy not found in /opt/ros/jazzy."
    [ -f "$REPO_ROOT/install/setup.bash" ] || fail "Workspace not built. Run 'colcon build --symlink-install' in $REPO_ROOT."
    if ! python3 -c "import cv2" 2>/dev/null; then
        fail "python3 cannot import cv2 (OpenCV). Install it with 'sudo apt install python3-opencv'.
If it is installed, a pip NumPy 2.x in ~/.local (pulled in by PX4 Tools/setup/ubuntu.sh via pandas)
probably shadows the system NumPy 1.x that python3-opencv is built against. Fix:
  python3 -m pip uninstall --break-system-packages -y numpy pandas && sudo apt install -y python3-pandas"
    fi
fi

if pgrep -x px4 >/dev/null || pgrep -f "gz sim" >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
    fail "A previous simulation is still running (px4 / gz sim / MicroXRCEAgent).
Two Gazebo servers with the same world interfere with each other. Stop it first:
  killall -9 px4 gz ruby MicroXRCEAgent"
fi

# ==========================================
# Cleanup: stop everything started by this script on Ctrl+C / exit
# ==========================================
cleanup() {
    echo -e "\n[1/2] Sending graceful shutdown signal..."
    trap - SIGINT EXIT # Disable the trap so it doesn't loop
    trap '' SIGTERM    # Survive our own 'kill 0' so the steps below still run

    # Gracefully terminate the processes started by this script (whole process group)
    kill 0

    # Give PX4 and Gazebo 2 seconds to save their log files
    sleep 2

    echo "[2/2] Hunting down lingering processes..."
    killall px4 gz ruby MicroXRCEAgent 2>/dev/null || true
    sleep 1
    # The Gazebo server sometimes ignores SIGTERM; a leftover one breaks the next run.
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -x MicroXRCEAgent 2>/dev/null || true

    echo "Cleanup complete. Goodbye!"
}
trap cleanup SIGINT SIGTERM EXIT

echo "========================================"
echo "1. Generating HydroLab World"
echo "========================================"
python3 "$SCRIPT_DIR/build_world.py"

echo ""
echo "========================================"
echo "2. Setting Environment Variables"
echo "========================================"
export PX4_GZ_WORLD=hydrolab_built
# x,y,z,roll,pitch,yaw — on start_base pad (top z=0.1 m; x500 legs ~0.012 m above spawn origin)
export PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0"
# The x500 airframe defaults to NAV_DLL_ACT=2, which blocks arming without a
# ground station ("Preflight Fail: No connection to the GCS"). Disable it so
# QGroundControl is optional. PX4 applies PX4_PARAM_* variables at startup.
export PX4_PARAM_NAV_DLL_ACT="${PX4_PARAM_NAV_DLL_ACT:-0}"
echo "PX4_ROOT is set to: $PX4_ROOT"
echo "PX4_GZ_WORLD is set to: $PX4_GZ_WORLD"
echo "PX4_GZ_MODEL_POSE is set to: $PX4_GZ_MODEL_POSE"
echo "HYDROLAB_CONFIG_DIR is set to: $HYDROLAB_CONFIG_DIR"

echo ""
echo "========================================"
echo "3. Launching PX4 SITL and Gazebo"
echo "========================================"
cd "$PX4_ROOT"
# Background jobs in a script get /dev/null as stdin; the PX4 shell (pxh) then
# hits EOF and redraws its prompt in a busy loop. Feed it a pipe that never closes.
sleep infinity | make px4_sitl gz_x500_gimbal &
sleep 20

echo ""
echo "========================================"
echo "4. Starting Micro XRCE-DDS Agent"
echo "========================================"
MicroXRCEAgent udp4 -p 8888 &
sleep 2

echo ""
echo "========================================"
echo "5. Starting QGroundControl (optional)"
echo "========================================"
if [ -f ~/QGroundControl.AppImage ]; then
    ~/QGroundControl.AppImage &
    sleep 20
else
    echo "QGroundControl not found at ~/QGroundControl.AppImage, skipping (optional)."
fi

echo ""
echo "========================================"
echo "6. Launching HydroLab mission"
echo "========================================"
if [ "$LAUNCH_MISSION" = "1" ]; then
    cd "$REPO_ROOT"
    source /opt/ros/jazzy/setup.bash
    ros2 launch hydrolab_mission system_launch.py &
else
    echo "LAUNCH_MISSION=0: mission not started. Start it in another terminal:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  ros2 launch hydrolab_mission system_launch.py config_dir:=$HYDROLAB_CONFIG_DIR"
fi

echo ""
echo "==========================================================="
echo "✅ All systems running!"
echo "🛑 Press [Ctrl+C] in this terminal to shut down everything."
echo "==========================================================="

wait
