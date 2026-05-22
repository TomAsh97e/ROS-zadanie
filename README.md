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

### Runtime config without rebuild

Mission and camera YAMLs were moved to:

`runtime_config/hydrolab_mission/`

To run with external config files (so YAML edits do not require `colcon build`):

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mateusz/AbsoluteEdge/SImulation_Hydrolab
source install/setup.bash
ros2 launch hydrolab_mission system_launch.py config_dir:=/home/mateusz/AbsoluteEdge/SImulation_Hydrolab/runtime_config/hydrolab_mission
```

You can also set it once via environment variable:

```bash
export HYDROLAB_CONFIG_DIR=/home/mateusz/AbsoluteEdge/SImulation_Hydrolab/runtime_config/hydrolab_mission
ros2 launch hydrolab_mission system_launch.py
```

Quick test (barrel drop only):

```bash
ros2 launch test_mission barrel_drop_test.launch.py
```
