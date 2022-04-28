#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from gui_app_class import GUIApp #import GUIApp class for the gui elements (button etc.) and their functionalities
from services_and_messages.msg import Echosensors
from services_and_messages.srv import WarningEchoTest

def echo_warning_direction(boolean_array):
    return 0

def callback_subscriber_echo_warning(msg_bool_list):
    gui_function_handler.change_colour_on_sensor_labels(msg_bool_list.echo_dir)


if __name__ == '__main__':

    rospy.init_node('wheelchair_robot_gui', anonymous=True)

    gui_function_handler = GUIApp()

    sub = rospy.Subscriber("/collision_warning_dir", Echosensors, callback_subscriber_echo_warning)

    gui_function_handler.run()


