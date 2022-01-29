#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseActionGoal
import time

if not rospy.has_param('move_base/DWAPlannerROS/max_vel_x'):
    rospy.set_param('move_base/DWAPlannerROS/max_vel_x', 0.5)

if not rospy.has_param('move_base/DWAPlannerROS/min_vel_x'):
    rospy.set_param('move_base/DWAPlannerROS/min_vel_x', 0)


def callback(config):
    pass
    #rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))


def goalSet(msg):
    msg = MoveBaseActionGoal()
    rospy.loginfo("Goal detected")
    client.update_configuration({"min_vel_x": 0})
    time.sleep(3)
    rospy.loginfo("velocity is changed")
    client.update_configuration({"min_vel_x": -0.5})



if __name__ == '__main__':
    rospy.init_node("prevent_initial_backwards_movement_node")

    client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30, config_callback=callback)
    sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goalSet)  # subscribe to
    rospy.spin()
