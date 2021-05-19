#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
import os

#creating a class used to make/connect the .kv file in order to run/show the gui
class GUIApp(MDApp):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)

        self.screen=Builder.load_file('/home/dennis/wheelchair-robot_catkin/src/wheelchair_robot/robot_gui/src/ros_robot_gui.kv')
        #if we had a second .kv file we would add it like this:
        #self.screen=Builder.load_file('/home/wheelchair-robot_catkin/src/wheelchair_robot/robot_gui/ros_robot_gui2.kv')

    def build(self):
        return self.screen

    #functions like a button press will be defined in this class
    def restart_pi(self,*args):
        os.system('systemctl reboot -i') #reboots system on button press
        


