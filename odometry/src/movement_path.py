#!/usr/bin/env python3

import rospy
import numpy as np
import math
from services_and_messages.msg import Position
from geometry_msgs.msg import Twist

offset = 512
throttling = 70


def callback_receive_position(msg):
    msg = Position
    return msg


def movement(x, y, t):
    msg_mov = Twist()
    msg_mov.linear.x = x
    msg_mov.linear.y = y
    msg_mov.angular.z = t
    pub.publish(msg_mov)


def query_linear_x(s_x):
    msg_pos = callback_receive_position()
    pos_x = msg_pos.linear.x
    pos_y = msg_pos.linear.y
    s = 0
    while abs(s) < abs(s_x):
        x = pos_x - msg_pos.linear.x
        y = pos_y - msg_pos.linear.y
        s = math.sqrt(x*x + y*y)
        if s_x < 0:
            movement(-5, 0, 0)
        else:
            movement(5, 0, 0)

def query_linear_x(s_y):
    msg_pos = callback_receive_position()
    pos_x = msg_pos.x
    pos_y = msg_pos.y
    s = 0
    while abs(s) < abs(s_y):
        x = pos_x - msg_pos.x
        y = pos_y - msg_pos.y
        s = math.sqrt(x*x + y*y)
        if s_y < 0:
            movement(0, -5, 0)
        else:
            movement(0, 5, 0)

def query_angular_z(s_t):
    msg_pos = callback_receive_position()
    pos_t = msg_pos.t
    s = 0
    while abs(s) < abs(s_t):
        s = pos_t - msg_pos.t

        if s_t < 0:
            movement(0, 0, -5)
        else:
            movement(0, 0, 5)


if __name__ == '__main__':
    rospy.init_node("joystick_movement")

    sub = rospy.Subscriber("/odom/position", Position, callback_receive_position)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    movement(5, 0, 0)

    #query_linear_x(1)
    #query_linear_x(-1)
    # query_angular_z(2*math.pi)
    rospy.spin()
