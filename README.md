# Wheelchair Robot

This Repository is used to program a wheelchair robot that will be used to autonomously drive older or disabled people with the help of SLAM and path planning. This project is part of a masters-degree-program.

Repository is updated and used by tulto and dennihub.

Date: 22.04.2021

author: tulto
author: dennihub

This project was divided into two parts: 
The first part is localization and path finding and the second part is more detailed about the SLAM algorithms.

## Hardware and Sofware
Als Hardware wird ein Raspberry Pi 4 4Gb oder 8Gb verwendet, auf dem Ubuntu 20.04 LTS läuft mit dem ROS System Noetic.

An installation guide of ROS can be seen within the link below:

http://wiki.ros.org/action/fullsearch/Installation/Ubuntu?action=fullsearch&context=180&value=linkto%3A%22Installation%2FUbuntu%22


To use this repository inside you´re system:
```
cd ~./catkin_ws/src
git clone https://github.com/tulto/wheelchair_robot.git
cd ..
catkin_make
```
