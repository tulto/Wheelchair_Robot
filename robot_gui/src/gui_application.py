#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from gui_app_class import GUIApp



if __name__ == '__main__':

    rospy.init_node('wheelchair_robot_gui', anonymous=True)

    GUIApp().run()

