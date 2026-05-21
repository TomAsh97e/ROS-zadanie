add a sweep down into the drone sweeping sequence. + 

review the searching sequence logic, i want the droen to stop sweeping whenever it sees the ball anywhere on camera, in that case it should immidiatelly try to position the ball on the center of the camera image  

cd ~/projects/Simulation_last_mile_logistics
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select last_mile_logistics_mission --symlink-install
source install/setup.bash
ros2 launch last_mile_logistics_mission system_launch.py
