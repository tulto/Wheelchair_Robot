## WCR-Launchfiles

### Launch
In order to launch all of the needed basic functions of the wheelchair robot - for example filtering and cutting the LiDAR-data, connecting to the arduino of the wheelchair robot and receiving as well of fusing the odometry data with an EKF filter - we only need to start one single launch file which will start all the other needed programs.
This launch file is the wcr_basics_launch.launch and can be started with the following command:
```
roslaunch wcr_launchfiles wcr_basics_launch.launch
```
