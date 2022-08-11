#!/bin/bash

echo "Starting roscore and Wheelchair-Robot Basics function!"

roscore & 

#sleep 12

#roslaunch wcr_launchfiles wcr_basics_launch.launch &

sleep 5

rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

#if you want to add other launch files to the script just add them here with < & Launchfile_Name >