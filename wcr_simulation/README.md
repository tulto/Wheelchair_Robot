## WCR-Simulation

### Installation
For using this simulation of the omnidirectional WCR you need the simulation environment Gazbeo, if you haven't installed it already.
```
sudo apt-get install ros-noetic-gazebo-ros
```
If your system asks for a gazebo_ros_2Dmap_plugin please also execute the following steps:
```
Go into your ROS-workspace most likely catkin_ws.

BEGIN:

cd ~/catkin_ws/src/

git clone https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin.git

cd ..

caktin_make
```

### Launch
With Gazebo installed there are differnt ways of launching the simulation of the wheelchair-robot (WCR).
One option is to start an "empty" simulation with only the WCR in it, which has ONE lidar-sensor:
```
roslaunch simulation-wcr_gazebo empty_wcr_simulation.launch
```
The second option is to launch an "empty" simulation with the WCR in it, which uses TWO lidar-sensors:
```
roslaunch simulation-wcr_gazebo empty_wcr_two_lidar_simulation.launch
```
And the last option is to start a simulation of the WCR with two lidar-sensors inside of a building called "nursing home":
```
roslaunch simulation-wcr_gazebo nursing_home_wcr_simulation.launch
```
