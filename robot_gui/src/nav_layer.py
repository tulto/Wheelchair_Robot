#!/usr/bin/env python3

import imp
import rospy
import numpy as np
import math
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# cancle msgs as string
def callback_cancel(msg):
    msg_stop = GoalID()
    msg_stop.stamp.nsecs = 0
    msg_stop.stamp.secs = 0
    msg_stop.id = ""
    pub_stop.publish(msg_stop)

def callback_goal(msg):

    goal = "/navigation_goal_stop_node/" 
    goal += msg.data

    pose = rospy.get_param(goal)
    
    msg_goal = PoseStamped()
    msg_goal.header.stamp = rospy.Time()
    msg_goal.header.stamp = rospy.Time()
    msg_goal.header.frame_id = "map"
    
    msg_goal.pose.position.x = pose[0]
    msg_goal.pose.position.y = pose[1]
    msg_goal.pose.position.z = pose[2]

    msg_goal.pose.orientation.x = pose[3]
    msg_goal.pose.orientation.y = pose[4]
    msg_goal.pose.orientation.z = pose[5]
    msg_goal.pose.orientation.w = pose[6]

    pub_goal.publish(msg_goal)


if __name__ == '__main__':
    rospy.init_node("navigation_goal_stop_node")

    sub_stop = rospy.Subscriber("/cancel_nav", String, callback_cancel)  # subscribe to /encoder data
    pub_stop = rospy.Publisher("/move_base/cancel", GoalID, queue_size=5)

    sub_goal = rospy.Subscriber("/goal_nav", String, callback_goal)  # subscribe to /encoder data
    pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)

    rospy.spin()
