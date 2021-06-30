#!/usr/bin/env python3

import rospy
import numpy as np
import math
from services_and_messages.msg import Encoder
from services_and_messages.msg import Position

radius = 203 / 2  # radius of the mecanum wheel
L = 740  # length of the vehicle
W = 920  # width of the vehicle
enc = 2750  # encoder count per turn

pos = np.array([[0], [0], [0]])  # starting position


# calculating encoder data to odometry
def callback_receive_encoder_data(msg):
    # setting up formula for calculating relative movement of vehicle
    encoder = np.array([[msg.encoder_wheel[0] * 2 * math.pi / enc], [msg.encoder_wheel[1] * 2 * math.pi / enc],
                        [msg.encoder_wheel[2] * 2 * math.pi / enc], [msg.encoder_wheel[3] * 2 * math.pi / enc]])
    relative_a = np.array([[-1, 1, -1, 1], [-1, -1, 1, 1], [-2 / (L + W), -2 / (L + W), -2 / (L + W), -2 / (L + W)]])
    rel_mov = radius / 4 * np.dot(relative_a, encoder)
    rospy.loginfo("relative movement")
    rospy.loginfo(rel_mov)

    # calculating absolute position based on starting point
    global pos
    absolute_a = np.array([[math.cos(pos[2][0] + rel_mov[2][0] / 2), math.sin(pos[2][0] + rel_mov[2][0] / 2), 0],
                           [math.sin(pos[2][0] + rel_mov[2][0] / 2), math.cos(pos[2][0] + rel_mov[2][0] / 2), 0],
                           [0, 0, 1]])
    position = pos + np.dot(absolute_a, rel_mov)
    pos = position
    rospy.loginfo("absolute position")
    rospy.loginfo(position)

    # publishing Position data to network
    msg_pos = Position()
    msg_pos.x = position[0][0]
    msg_pos.y = position[1][0]
    msg_pos.turn = position[2][0]
    pub.publish(msg_pos)


if __name__ == '__main__':
    rospy.init_node("odometry_encoder")

    sub = rospy.Subscriber("/encoder", Encoder, callback_receive_encoder_data)  # subscribe to /encoder data
    pub = rospy.Publisher("/position/encoder", Position, queue_size=10)
    rate = rospy.Rate(5)
    rospy.spin()
