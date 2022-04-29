#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from gui_app_class import GUIApp #import GUIApp class for the gui elements (button etc.) and their functionalities
from services_and_messages.msg import Echosensors
from services_and_messages.msg import TOF_sensor

bool_list = []
#bool_list_col = []
#bool_list_stair = []

def echo_warning_direction(boolean_array):
    return 0

def callback_subscriber_echo_warning(msg_bool_list):
    global bool_list_col
    bool_list_col=msg_bool_list.echo_dir
    bool_list[0] = (bool_list_col[0] or bool_list_stair[0])
    bool_list[1] = (bool_list_col[1] or bool_list_stair[1])
    bool_list[2] = (bool_list_col[2] or bool_list_stair[2])
    bool_list[3] = (bool_list_col[3] or bool_list_stair[3])
    gui_function_handler.change_colour_on_sensor_labels(bool_list)

def callback_subscriber_stair_warning(msg_bool_list):
    global bool_list_stair
    bool_list_stair=msg_bool_list.stair_warning_dir
    bool_list[0] = (bool_list_col[0] or bool_list_stair[0])
    bool_list[1] = (bool_list_col[1] or bool_list_stair[1])
    bool_list[2] = (bool_list_col[2] or bool_list_stair[2])
    bool_list[3] = (bool_list_col[3] or bool_list_stair[3])
    gui_function_handler.change_colour_on_sensor_labels(bool_list)
    

if __name__ == '__main__':

    rospy.init_node('wheelchair_robot_gui', anonymous=True)

    gui_function_handler = GUIApp()

    sub_col = rospy.Subscriber("/collision_warning_dir", Echosensors, callback_subscriber_echo_warning)
    sub_stair = rospy.Subscriber("/stair_warning_dir", TOF_sensor, callback_subscriber_stair_warning)

    gui_function_handler.run()


