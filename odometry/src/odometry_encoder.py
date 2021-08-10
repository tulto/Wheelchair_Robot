#!/usr/bin/env python3

import rospy
import numpy as np
import math
from services_and_messages.msg import Encoder
from services_and_messages.msg import Position
from nav_msgs.msg import Odometry

radius = 0.203 / 2  # radius of the mecanum wheel
L = 0.740  # length of the vehicle
W = 0.920  # width of the vehicle
enc = 2750  # encoder count per turn

pos = np.array([[0], [0], [0]])  # starting position


# calculating encoder data to odometry
def callback_receive_encoder_data(msg):
    # setting up formula for calculating relative movement of vehicle
    encoder = np.array([[msg.encoder_wheel[0] * 2 * math.pi / enc], [msg.encoder_wheel[1] * 2 * math.pi / enc],
                        [msg.encoder_wheel[2] * 2 * math.pi / enc], [msg.encoder_wheel[3] * 2 * math.pi / enc]])
    relative_a = np.array([[1, 1, 1, 1], [1, -1, -1, 1], [2 / (L + W), -2 / (L + W), 2 / (L + W), -2 / (L + W)]])
    rel_mov = radius / 4 * np.dot(relative_a, encoder)
    rel_mov = rel_mov * np.array([[1], [0.957], [1.042]])  # Bias factor-error calculated

    # calculating absolute position based on starting point
    global pos
    absolute_a = np.array([[math.cos(pos[2][0] + rel_mov[2][0] / 2), math.sin(pos[2][0] + rel_mov[2][0] / 2), 0],
                           [math.sin(pos[2][0] + rel_mov[2][0] / 2), math.cos(pos[2][0] + rel_mov[2][0] / 2), 0],
                           [0, 0, 1]])
    position = pos + np.dot(absolute_a, rel_mov)
    pos = position

    # publishing Position data to network
    msg_pos = Position()
    msg_pos.x = position[0][0]
    msg_pos.y = position[1][0]
    msg_pos.turn = position[2][0]
    pub_pos.publish(msg_pos)

    # publish velocity
    msg_odom = Odometry()
    msg_odom.pose.pose.position.x = position[0][0]
    msg_odom.pose.pose.position.y = position[1][0]

    msg_odom.twist.twist.linear.x = rel_mov[0][0] / (msg.time / 100000)
    msg_odom.twist.twist.linear.y = rel_mov[1][0] / (msg.time / 100000)
    msg_odom.twist.twist.angular.z = rel_mov[2][0] / (msg.time / 100000)
    msg_odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                 0, 0.1, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0.1]
    pub_vel.publish(msg_odom)


if __name__ == '__main__':
    rospy.init_node("odometry_encoder")

    sub = rospy.Subscriber("/encoder", Encoder, callback_receive_encoder_data)  # subscribe to /encoder data
    pub_pos = rospy.Publisher("/odom/position", Position, queue_size=10)
    pub_vel = rospy.Publisher("/odom/data", Odometry, queue_size=10)
    rate = rospy.Rate(5)
    rospy.spin()
