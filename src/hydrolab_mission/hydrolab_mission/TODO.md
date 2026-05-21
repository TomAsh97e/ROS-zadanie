add a sweep down into the drone sweeping sequence. + 

tune the gimbal pid loop. 
tune the hover pid loop. 

tune the point estimation (my idea is to actually move teh point a bit closer than calculated to prevent overshoot)


review the searching sequence logic, i want the droen to stop sweeping whenever it sees the ball anywhere on camera, in that case it should immidiatelly try to position the ball on the center of the camera image  






check if the lidar was added correctly, if yes - add it to the readme and repo 

incorporate lidar into the program by writing another node that will do this calculations and also use this tf2 library or sth t ohandle transofrmations.

consult if i should continue with the balls or switch to hydrolab

do the whole basic hydrolab mission. 

## Build & launch

```bash
cd /home/mateusz/AbsoluteEdge/SImulation_Hydrolab
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hydrolab_mission --symlink-install
source install/setup.bash
ros2 launch hydrolab_mission system_launch.py
```
