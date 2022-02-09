#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def callback(msg):
    msg_odom = Odometry()
    msg_odom.header = msg.header
    msg_odom.child_frame_id = "base_link"
    msg_odom.pose = msg.pose
    pub.publish(msg_odom)  # publishing data to /imu/data

if __name__ == '__main__':

    rospy.init_node("odometry_amcl_node")

    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)

    pub = rospy.Publisher("/amcl/odometry", Odometry, queue_size=10)

    rospy.spin()