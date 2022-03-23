#!/usr/bin/env python3
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
import sys
import json
from collections import deque

import time


def callback(data):
    global xAnt
    global yAnt
    global cont

    pose = PoseStamped()

    pose.header.frame_id = "odom"
    pose.pose.position.x = float(data.pose.pose.position.x)
    pose.pose.position.y = float(data.pose.pose.position.y)
    pose.pose.position.z = float(data.pose.pose.position.z)
    pose.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        # Published the msg

    cont = cont + 1

    rospy.loginfo("Hit: %i" % cont)
    if cont > max_append:
        path.poses.pop(0)

    pub.publish(path)

    xAnt = pose.pose.orientation.x
    yAnt = pose.pose.position.y
    return path


def callback_2(data):
    global xAnt2
    global yAnt2
    global cont2

    pose2 = PoseStamped()

    pose2.header.frame_id = "odom"
    pose2.pose.position.x = float(data.pose.pose.position.x)
    pose2.pose.position.y = float(data.pose.pose.position.y)
    pose2.pose.position.z = float(data.pose.pose.position.z)
    pose2.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose2.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose2.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose2.pose.orientation.w = float(data.pose.pose.orientation.w)

    if (xAnt2 != pose2.pose.position.x and yAnt2 != pose2.pose.position.y):
        pose2.header.seq = path.header.seq + 1
        path2.header.frame_id = "odom"
        path2.header.stamp = rospy.Time.now()
        pose2.header.stamp = path.header.stamp
        path2.poses.append(pose2)
        # Published the msg

    cont2 = cont2 + 1
    if cont2 > max_append:
        path2.poses.pop(0)

    pub_2.publish(path2)

    xAnt2 = pose2.pose.orientation.x
    yAnt2 = pose2.pose.position.y
    return path2


def callback_3(data):
    global xAnt3
    global yAnt3
    global cont3

    pose3 = PoseStamped()

    pose3.header.frame_id = "odom"
    pose3.pose.position.x = float(data.pose.pose.position.x)
    pose3.pose.position.y = float(data.pose.pose.position.y)
    pose3.pose.position.z = float(data.pose.pose.position.z)
    pose3.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose3.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose3.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose3.pose.orientation.w = float(data.pose.pose.orientation.w)

    if (xAnt3 != pose3.pose.position.x and yAnt3 != pose3.pose.position.y):
        pose3.header.seq = path.header.seq + 1
        path3.header.frame_id = "odom"
        path3.header.stamp = rospy.Time.now()
        pose3.header.stamp = path.header.stamp
        path3.poses.append(pose3)
        # Published the msg

    cont3 = cont3 + 1
    if cont3 > max_append:
        path3.poses.pop(0)

    pub_3.publish(path3)

    xAnt2 = pose3.pose.orientation.x
    yAnt2 = pose3.pose.position.y
    return path3

if __name__ == '__main__':
    # Initializing global variables
    global xAnt
    global yAnt
    global cont
    xAnt = 0.0
    yAnt = 0.0
    cont = 0

    global xAnt2
    global yAnt2
    global cont2
    xAnt2 = 0.0
    yAnt2 = 0.0
    cont2 = 0

    global xAnt3
    global yAnt3
    global cont3
    xAnt3 = 0.0
    yAnt3 = 0.0
    cont3 = 0

    # Initializing node
    rospy.init_node('path_plotter')

    # Rosparams set in the launch (can ignore if running directly from bag)
    # max size of array pose msg from the path
    if not rospy.has_param("~max_list_append"):
        rospy.logwarn('The parameter max_list_append dont exists')
    max_append = rospy.set_param("~max_list_append", 1000000)
    max_append = 1000000
    if not (max_append > 0):
        rospy.logwarn('The parameter max_list_append is not correct')
        sys.exit()

    pub = rospy.Publisher('/path', Path, queue_size=1)
    pub_2 = rospy.Publisher('/path_encoder', Path, queue_size=1)
    pub_3 = rospy.Publisher('/path_hector', Path, queue_size=1)

    path = Path()
    msg = Odometry()

    path2 = Path()
    msg2 = Odometry()

    path3 = Path()
    msg3 = Odometry()

    # Subscription to the required odom topic (edit accordingly)
    msg = rospy.Subscriber('/odometry/filtered', Odometry, callback)
    msg2 = rospy.Subscriber('/odom/data', Odometry, callback_2)
    msg3 = rospy.Subscriber('/odom/hector', Odometry, callback_3)
    rate = rospy.Rate(5)  # 30hz

    try:
        while not rospy.is_shutdown():
            # rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
