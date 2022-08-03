## LiDAR-Filtering

### Explenation
This package contains launch files as well as program which are used to filter, change and/or check the data measured by the LiDAR-Sensors of the wheelchair robot.

### Installation
A lot of packages and launch files which are used in this package need the laser_filters ROS-package.
For installing this package use the following command:
```
sudo apt-get install ros-noetic-laser-filters
```
More information about the laser-filters ROS-package can be found on: http://wiki.ros.org/laser_filters

### Launch
There are several different launch files in this package.
Cutting the LiDAR-Scans of the wheelchair robot simulation (with two lidar-sensors) you need to start the merging_lidar_laser.launch file.
```
roslaunch lidar-filtering two_lidar_simulation_setup.launch
```

For the REAL wheelchair robot (or the robot you use) you need to start several launch files.
```
roslaunch lidar-filtering real_robot_laser_cutter.launch
roslaunch lidar-filtering real_wcr_lidar_merging.launch
```
BUT there also is a launch file which is located in the wcr_launchfiles package, which will start all needed (other) launch files for the basic functions for the wheelchair robot to function. These basic functions include the LiDAR-scan-filtering as well as the connectino to the arduino of the robot and the EKF-filter for the odometry data.

### Programs

The source folder of this package contains two different pyhton programs, which are both dedicated to changing or filtering the LiDAR sensor data.

lidar_average_window.py is a program which implements a moving average filter over the LiDAR-data of consecutive scans.
```
src/lidar_average_window.py 
```

change_laser_data_max_range.py is a program which changes the maximum "theoretical" range a LiDAR-Sensor can measure.
```
src/change_laser_data_max_range.py 
```
