#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from gui_app_class import GUIApp
from services_and_messages.msg import Echosensors

def echo_warning_direction(boolean_array):
    return 0

def callback_subscriber_echo_warning(msg_bool_array):
    msg_bool_array.echo_dir 


if __name__ == '__main__':

    rospy.init_node('wheelchair_robot_gui', anonymous=True)

    sub = rospy.Subscriber("/echo_sensor", Echosensors, callback_subscriber_echo_warning )

    GUIApp().run()

