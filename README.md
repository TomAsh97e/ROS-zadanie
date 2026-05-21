# Simulation_Hydrolab

PX4 SITL + Gazebo HydroLab world with ROS 2 Python mission nodes.

## Shutdown

```bash
killall -9 px4 gz ruby MicroXRCEAgent
```

## ROS 2 workspace

Terminal 1 — simulation (PX4, Gazebo, Micro XRCE-DDS Agent):

```bash
cd SimWorld_hydrolab
./launch_envhydrolab.sh
```

Terminal 2 — mission nodes:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mateusz/AbsoluteEdge/SImulation_Hydrolab
colcon build --symlink-install
source install/setup.bash
ros2 launch hydrolab_mission system_launch.py
```

Quick test (barrel drop only):

```bash
ros2 launch test_mission barrel_drop_test.launch.py
```
