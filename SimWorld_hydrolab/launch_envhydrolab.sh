#!/bin/bash
set -e

# ==========================================
# 1. FIXED CLEANUP TRAP
# Disables itself immediately so it doesn't loop forever!
# ==========================================
cleanup() {
    echo -e "\n[1/2] Sending graceful shutdown signal..."
    trap - SIGINT SIGTERM EXIT # Disable the trap so it doesn't loop
    
    # 1. Ask nicely: Gracefully terminate only the processes started by this script
    kill 0 
    
    # Give PX4 and Gazebo 2 seconds to save their log files
    sleep 2 
    
    echo "[2/2] Hunting down lingering zombie processes..."
    # 2. Force cleanup: Run a standard killall (WITHOUT the destructive -9 flag)
    # The '2>/dev/null || true' hides ugly error messages if the processes are already dead
    killall px4 gz ruby MicroXRCEAgent 2>/dev/null || true
    
    echo "Cleanup complete. Goodbye!"
}
trap cleanup SIGINT SIGTERM EXIT
# Define standard paths
PX4_ROOT=/home/mateusz/AbsoluteEdge/SImulationENV/PX4-Autopilot
SOFTWARE_ROOT=/home/mateusz/AbsoluteEdge/Software/PX4

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
MicroXRCEAgent udp4 -p 8888 &
sleep 2

echo ""
echo "========================================"
echo "5. Starting QGroundControl"
echo "========================================"
if [ -f ~/QGroundControl.AppImage ]; then
    ~/QGroundControl.AppImage &
else
    echo "WARNING: QGroundControl not found at ~/QGroundControl.AppImage"
fi
sleep 5


echo ""
echo "========================================"
echo "7. Launching OpenCV Camera Viewer"
echo "========================================"
cd "$SOFTWARE_ROOT"
# 2. FIXED PYTHON EXECUTION
# Directly targeting the python engine inside the venv avoids all activation bugs!
source /opt/ros/jazzy/setup.bash  # Change 'humble' if you use a different ROS 2 version
source /home/mateusz/AbsoluteEdge/SImulation_Hydrolab/install/setup.bash  # Source your ROS 2 workspace (if you have one)

# 2. Launch the Pilot and State Machine in the background (&)
ros2 launch hydrolab_mission system_launch.py &


echo ""
echo "==========================================================="
echo "✅ All systems running!"
echo "📡 Live Camera Feed should pop up momentarily."
echo "🛑 Press [Ctrl+C] in this terminal to shut down everything."
echo "🚀 To take off, open a NEW terminal and run takeoff.py"
echo "==========================================================="

wait
