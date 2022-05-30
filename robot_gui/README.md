## Robot-GUI

### Installation
This is a simple GUI programmed with python and the kivymd library.
In order to use this application you need to install the kivymd-python library.
```
pip install kivymd
```
Further infomation can be found on https://kivymd.readthedocs.io/en/latest/getting-started/ .

### Launch
For launching the robot-gui simply type:
```
rosrun robot_gui gui_application.py
```


### Navigation Layer

Lauch, when navigation is already used, over:
```
roslaunch robot_gui nav_layer.launch
```

different locations can be defined via the launchfile and the parameters (examples in `nav_layer.lauch`).
If now a string is sent via the topic `/goal_nav` that is similar to the previously created parameters, this parameter is passed on as a position in move_base.

Cancel navigation, on the other hand, works via the `/cancle_nav` topic, which simply needs to be sent an empty string.

On the other hand, the status of the navigation can be obtained directly from `/move_base/status`. The message is `actionlib_msgs/GoalStatusArray` with the important part `msg.status_list.status`. A number will be sent with 0 = PENDING, 1 = ACTIVE, 2 = PREEMPTED, 3 = SUCCEDED, 4 = ABORTED, 5 = REJECTED, 6 = PREEMTING, 7 = RECALLING, 8 = RECALLED, 9 = LOST.

All of this can be tested inside of the simulation which can be launched over:
```
roslaunch navigation navigation_nursing_home_sim.launch
```
