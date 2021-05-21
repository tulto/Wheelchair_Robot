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

    #functions like a button press will be defined in this class (see below)
    def restart_pi(self,*args):
        os.system('systemctl reboot -i') #reboots system on button press

    def testing(self,*args):
        print("testbutton has been pressed!")
        self.screen.ids.front_sensor.md_bg_color=[1,0,0,1]
        self.screen.ids.front_sensor_label.text="Sensor blockiert"

    def change_colour(self, boolean_list):
        #self.screen gives access to the root of the .kv file (so basically to FloatLayout)
        #with the self.screen.ids.<id_name>.text="change text here"
        
        if boolean_list[0]:
            self.screen.ids.front_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.front_sensor_label.text="Sensor blockiert"
        else:
            self.screen.ids.front_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.front_sensor_label.text="Sensor frei"


        if boolean_list[1]:
            self.screen.ids.left_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.left_sensor_label.text="Sensor blockiert"
        else:
            self.screen.ids.left_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.left_sensor_label.text="Sensor frei"
            

        if boolean_list[2]:
            self.screen.ids.right_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.right_sensor_label.text="Sensor blockiert"
        else:
            self.screen.ids.right_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.right_sensor_label.text="Sensor frei"


        if boolean_list[3]:
            self.screen.ids.back_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.back_sensor_label.text="Sensor blockiert"
        else:
            self.screen.ids.back_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.back_sensor_label.text="Sensor frei"
        


