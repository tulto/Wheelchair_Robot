#!/usr/bin/env python3

import csv
import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

file_imu = open('imu.csv', 'w')
file_odom = open('odom.csv', 'w')

def calback_odom(msg):
    with open('odom.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter = ';')
        writer.writerow([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w, msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                         msg.twist.twist.angular.z])


def calback_imu(msg):
    with open('imu.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        writer.writerow([msg.orientation.z, msg.orientation.w,
                         msg.angular_velocity.z,
                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])


if __name__ == '__main__':

    rospy.init_node("odometry_csv_node")

    sub = rospy.Subscriber("/odom/data", Odometry, calback_odom)
    sub = rospy.Subscriber("/imu/data", Imu, calback_imu)

    rospy.spin()