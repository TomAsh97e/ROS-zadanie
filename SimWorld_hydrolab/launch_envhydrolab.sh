#!/bin/bash
set -e

# ==========================================
# 1. FIXED CLEANUP TRAP
# Disables itself immediately so it doesn't loop forever!
# ==========================================
cleanup() {
    GRACEFUL_SHUTDOWN_WAIT_TIME_SEC=10
    trap - SIGINT SIGTERM EXIT # Disable the trap so it doesn't loop
    
    echo -e "\nSending graceful shutdown signals...\n"
    killall px4 gz ruby MicroXRCEAgent QGroundControl 2>/dev/null || true
    
    
    echo -e "\nWaiting for gracefull shutdown ($GRACEFUL_SHUTDOWN_WAIT_TIME_SEC s)...\n"
    sleep $GRACEFUL_SHUTDOWN_WAIT_TIME_SEC
    
    echo -e "\nForcing shutdown...\n"
    killall -9 px4 gz ruby MicroXRCEAgent QGroundControl 2>/dev/null || true
    
    echo -e "Cleanup complete. Goodbye!\n"
    kill 0
}
trap cleanup SIGINT SIGTERM EXIT

# ADJUST THESE PATHS TO YOUR ENVIRONMENT!
PX4_ROOT=~/repos/PX4-Autopilot
WORKSPACE_ROOT=~/repos/Absolute-Edge-ROS
Q_GROUND_CONTROL_PATH=~/QGroundControl.AppImage

echo "========================================"
echo "1. Generating Randomized HydroLab World"
echo "========================================"
python3 build_world.py

echo ""
echo "========================================"
echo "2. Setting Environment Variables"
echo "========================================"
export PX4_GZ_WORLD=hydrolab_built
# x,y,z,roll,pitch,yaw — on start_base pad (top z=0.1 m; x500 legs ~0.012 m above spawn origin)
export PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0"
echo "PX4_GZ_WORLD is set to: $PX4_GZ_WORLD"
echo "PX4_GZ_MODEL_POSE is set to: $PX4_GZ_MODEL_POSE"

echo ""
echo "========================================"
echo "3. Launching PX4 SITL and Gazebo"
echo "========================================"
cd "$PX4_ROOT"
make px4_sitl gz_x500_gimbal &
sleep 20

echo ""
echo "========================================"
echo "4. Starting Micro XRCE-DDS Agent"
echo "========================================"
source "$WORKSPACE_ROOT/install/setup.bash"
MicroXRCEAgent udp4 -p 8888 &
sleep 2

echo ""
echo "========================================"
echo "5. Starting QGroundControl"
echo "========================================"
if [ -f $Q_GROUND_CONTROL_PATH ]; then
    $Q_GROUND_CONTROL_PATH &
else
    echo "WARNING: QGroundControl not found at $Q_GROUND_CONTROL_PATH"
fi
sleep 20


echo ""
echo "========================================"
echo "6. Launching ROS2 mission"
echo "========================================"
export HYDROLAB_CONFIG_DIR="$WORKSPACE_ROOT/runtime_config/hydrolab_mission"
# Directly targeting the python engine inside the venv avoids all activation bugs!
source /opt/ros/jazzy/setup.bash  # Change 'humble' if you use a different ROS 2 version
# Launch the ROS mission in the background (&)
ros2 launch hydrolab_mission system_launch.py &


echo ""
echo "==========================================================="
echo "✅ All systems running!"
echo "🛑 Press [Ctrl+C] in this terminal to shut down everything."
echo "==========================================================="

wait
