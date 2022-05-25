#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from gui_app_class import GUIApp #import GUIApp class for the gui elements (button etc.) and their functionalities
from services_and_messages.msg import Echosensors
from services_and_messages.msg import TOF_sensor
from std_msgs.msg import String
from compare_list_class import Compare
from actionlib_msgs.msg import GoalStatusArray

def callback_subscriber_echo_warning(msg_bool_list):
    
    compare.give_list_col(msg_bool_list.echo_dir)
    compare.change_colour(gui_function_handler)


def callback_subscriber_stair_warning(msg_bool_list):
 
    compare.give_list_stair(msg_bool_list.stair_warning_dir)
    compare.change_colour(gui_function_handler)

def callback_subscriber_active(msg_active):
    if len(msg_active.status_list) == 0:
        pass
    #print("helo")
    #print("test" + str(msg_active.status_list[0].status ))
    gui_function_handler.temporary_button_color(msg_active.status_list[0].status)

def callback_goal(msg):
    gui_function_handler.voice_button_change(msg.data)
    
    

if __name__ == '__main__':

    rospy.init_node('wheelchair_robot_gui', anonymous=True)

    gui_function_handler = GUIApp()

    compare = Compare()

    sub_col = rospy.Subscriber("/collision_warning_dir", Echosensors, callback_subscriber_echo_warning)
    sub_stair = rospy.Subscriber("/stair_warning_dir", TOF_sensor, callback_subscriber_stair_warning)
    sub_mb= rospy.Subscriber("/move_base/status", GoalStatusArray, callback_subscriber_active)
    sub_nav= rospy.Subscriber("/nav_goal", String, callback_goal)

    gui_function_handler.run()


