## AMCL_Launch

### Installation
In order to use amcl we need to install the amcl ros package with the following command:
```
sudo apt install ros-noetic-amcl
```
### Launch
In order to launch the amcl algorithm with all parameters used for the wheelchair robot, you need to type the following command:
```
roslaunch amcl_localization amcl_launch.launch
```

### Parameters
All parameters used for the AMCL-algorithmn can be found in in the launch file 
```
launch/amcl_launch.launch
```
which is located inside this package. All the parameters for the AMCL-localization-algorithmn have been empirically determnied and tested in real world conditions.
More information about the AMCL-algorithm as well as information about the corresponding parameters can be found on: http://wiki.ros.org/amcl

