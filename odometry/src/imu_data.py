#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu


# calculating encoder data to odometry
def callback_receive_imu_data(msg):
    msg_imu = Imu()

    msg_imu.header = msg.header
    deg_z = math.sin(math.asin(msg.orientation.z) * 2)
    deg_w = math.sin(math.asin(msg.orientation.w) * 2)
    if deg_z > 1:
        deg_z = deg_z - 2*3.14
    elif deg_z < -1
        deg_z = deg_z + 2*3.14
    if deg_w > 1:
        deg_w = deg_w - 2*3.14
    elif deg_w < -1
        deg_w = deg_w + 2*3.14

    msg_imu.orientation.z = deg_z
    msg_imu.orientation.w = deg_w
    msg_imu.orientation_covariance = [0.1, 0, 0,
                                      0, 0.1, 0,
                                      0, 0, 0.1]
    msg_imu.angular_velocity.x = msg.angular_velocity.x / 29.4437
    msg_imu.angular_velocity.y = msg.angular_velocity.y / 29.4437
    msg_imu.angular_velocity.z = msg.angular_velocity.z / 29.4437
    msg_imu.angular_velocity_covariance = [0.01, 0, 0,
                                           0, 0.01, 0,
                                           0, 0, 0.01]
    msg_imu.linear_acceleration = msg.linear_acceleration
    msg_imu.linear_acceleration_covariance = [0.2, 0, 0,
                                              0, 0.2, 0,
                                              0, 0, 0.2]
    pub.publish(msg_imu)


if __name__ == '__main__':
    seq = 0
    rospy.init_node("odometry_imu")

    sub = rospy.Subscriber("/imu/data", Imu, callback_receive_imu_data)  # subscribe to /encoder data
    pub = rospy.Publisher("/imu/data_2", Imu, queue_size=10)
    rospy.spin()
