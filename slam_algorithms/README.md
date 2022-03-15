## SLAM-Algorithms

### Installation
In order to use all of the four different SLAM-Algorithms the following packages need to be installed.
NOTE: The part with "-noetic-" changes depending on the ROS distrubution you are using.

Hector-SLAM:
(More Information about Hector-SLAM on | http://wiki.ros.org/hector_mapping )
```
sudo apt-get install ros-noetic-hector-mapping
```
Karto-SLAM:
(More Information about Karto-SLAM on | http://wiki.ros.org/slam_karto )
```
sudo apt-get install ros-noetic-slam-karto
```
Gmapping:
(More Information about Gmapping-SLAM on | http://wiki.ros.org/gmapping )
```
sudo apt-get install ros-noetic-slam-gmapping
```
The last used SLAM-Algorithm is (Google-)Cartographer. The installation of Cartographer is a little bit more complicated. 
You can look up how to install cartographer on: https://google-cartographer.readthedocs.io/en/latest/ .

### Preliminary work
Before you can use the Algorithms which use two-lidar sensors it is necessary that you start a merger which merges two lidar-scans into one lidar-scan.
This can be started with:
```
roslaunch slam_algorihtms merging_lidar_laser.launch
```
for the simulation of the robot. For the real robot you need to start:
```
roslaunch wcr_launchfiles lidar_setup_wcr.launch
```


### Launch
There are different launch files which can be started depending on if you are using the Gazebo Simulation of the WCR (also provided in this repository) or if you are using a real robot.
In this case the real robot used is the wheelchair-robot of this project. Also there are different launch files for the four different SLAM-Algorithms.
In the following there is an example for a launch file.
NOTE: You can change the SLAM-Parameters for all SLAM-Algorihtms, except Cartographer where you need to cange the .lua files, in their launch files.
All possibilitys for starting Hector-SLAM are shown below. For the other SLAM-Algorithms you just have to change "hector-" to for example to "karto-".

Hector-SLAM-launch file for WCR-simulation with two lidar sensors.
```
roslaunch slam_algorihtms hector_slam_wcr_two_lidar.launch
```
Hector-SLAM-launch file for the real WCR-robot.
```
roslaunch slam_algorihtms hector_slam_real_robot.launch
```

### Param
All SLAM-Algorithms have different parameters which can be changed in order to get a (hopefully) better result of the generated map.
These SLAM-parameters can be changed through the according launch file for Hector-, Gmapping-, and Karto-slam.
For Cartorapher you need to change the .lua files in the config folder.

More infomration about the changeable parameters of each algorithm can be found on the corresponding wiki.ros.org page | See links abouve.
