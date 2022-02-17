## Odometry

### Installation
These files are for getting encoder-odometry and Sensorfusion over an extended Kalman Filter.
To get the Sensordata from the arduino to ROS you need "rosserial"
```
sudo apt install ros-noetic-rosserial
```
Some of the Files depend on costom Messages inside "services_and_messages" so you need to get the costom messages inside Arduino Library of ROS (need to locate arduino libraries in your system):
```
rosrun rosserial_arduino make_libraries.py ~/libraries/ros_lib
```
For use you also need a ROS Package named "robot-localisation" so to install it inside ROS:
```
sudo apt install ros-noetic-robot_localization
```



### Launch
There is only one Launch file to activate everything inside odometry named odom.launch it starts rosserial (get sensordata from Arduino), encoder-odomertry (calculate odometry based on endcoder data) and robot-localisation for  Sensorfusion
```
roslaunch odometry odom.launch
```


### Param
The Param you see below are also the default values if not defined

Covarianz matrix for IMU data[3][3]:
```
<rosparam param="orientation_covariance">[0.1,0.,0.,  0.,0.1,0.,  0.,0.,0.1]</rosparam>
<rosparam param="gyro_covariance">[0.1,0.,0.,  0.,0.1,0.,  0.,0.,0.1]</rosparam>
<rosparam param="linear_covariance">[0.4,0.,0.,  0.,0.4,0.,  0.,0.,0.4]</rosparam>
```

Covarianz matrix for encoder Odometry[6][6]:
```
<rosparam param="pose_covariance">[0.01,0,0,0,0,0,  0,0.01,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0.1] </rosparam>
<rosparam param="twist_covariance">[0.2,0,0,0,0,0,  0,0.2,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0.2] </rosparam>
```

There is also inside ```odom.launch``` a movementoption for ``movement_joystick.py`` but the analogue joystick is at moment not connected to the arduino so it wouud get wrong values to work with -> not activated (to activate remove ``?`` inside ``odom.launch``. 

parameters of robot localisation are found in:
http://docs.ros.org/en/noetic/api/robot_localization/html/configuring_robot_localization.html

### Sensorfusion
This Sensorfusion is for the fusion between IMU and Encoder-odometry data (important to get the covarianz matrix right)

Fusing encoder-odometry (/odom/data) twice better flexibility if orientation should be absolute or differential:
- relative x and relative y movement (it needs to be differential = true)
- absolute orientation (yawn) 

the next fusion is about the IMU (/imu/data) Data where you get absolute orientation, relative orientation and acceleration.
The acceleration is with gavitational pull  (needed to set the ```param remove_gravitational_acceleration: true``` inside ekf_template.yaml)


