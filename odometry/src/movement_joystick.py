#!/usr/bin/env python3

import rospy
import numpy as np
import math
from services_and_messages.msg import Joystick
from geometry_msgs.msg import Twist

offset = 512
throttling = 70


def callback_receive_joystick(msg):
    if msg.button:
        msg_mov = Twist()
        msg_mov.linear.x = (msg.x - offset) / throttling
        msg_mov.linear.y = (msg.y - offset)/ throttling
        msg_mov.angular.z = (msg.t - offset) / throttling
    if not msg.button:
        msg_mov = Twist()
        msg_mov.linear.x = (msg.x - offset) / throttling
        msg_mov.linear.y = (msg.t - offset) / throttling
        msg_mov.angular.z = (msg.y - offset) / throttling

    pub.publish(msg_mov)


if __name__ == '__main__':
    rospy.init_node("joystick_movement")

    sub = rospy.Subscriber("/movement/joystick", Joystick, callback_receive_joystick)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.spin()
