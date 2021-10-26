#!/usr/bin/env python3

import rospy
import numpy as np
import math
from services_and_messages.msg import Joystick
from geometry_msgs.msg import Twist

offset = 512
zero = 0

if not rospy.has_param('/movement_joystick/deadzone'):
    rospy.set_param('/movement_joystick/deadzone', 50)

if not rospy.has_param('/movement_joystick/speed'):
    rospy.set_param('/movement_joystick/speed', 70)

speed = rospy.get_param('/movement_joystick/speed')
deadzone = rospy.get_param('/movement_joystick/deadzone')


def callback_receive_joystick(msg):
    global zero
    if msg.button:
        msg_mov = Twist()
        msg_mov.linear.x = (msg.x - offset) / throttling
        msg_mov.linear.y = (msg.y - offset) / throttling
        msg_mov.angular.z = (msg.t - offset) / throttling
    if not msg.button:
        msg_mov = Twist()
        msg_mov.linear.x = (msg.x - offset) / throttling
        msg_mov.linear.y = (msg.t - offset) / throttling
        msg_mov.angular.z = (msg.y - offset) / throttling

    # only send cmd_vel if joystick is not used
    if abs(msg.x - offset) > deadzone or abs(msg.y - offset) > deadzone or abs(msg.z - offset) > deadzone or zero == 0:
        pub.publish(msg_mov)
        zero = 0
        # sending once zero to cmd_vel
        if abs(msg.x - offset) < deadzone and abs(msg.y - offset) < deadzone and abs(msg.z - offset) < deadzone:
            zero = 1


if __name__ == '__main__':
    rospy.init_node("movement_joystick")

    sub = rospy.Subscriber("/movement/joystick", Joystick, callback_receive_joystick)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.spin()
