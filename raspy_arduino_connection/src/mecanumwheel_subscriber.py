#!/usr/bin/env python3

import rospy
from services_and_messages.msg import Encoder


def callback_mecanum_wheel_data(msg):
    rospy.loginfo("Encoder data received: ")
    rospy.loginfo("Left-Front-Wheel turning: " + msg.lfw_dir + "| Encoder count is " + msg.lfw_count)
    rospy.loginfo("Left-Back-Wheel turning: " + msg.lbw_dir + "| Encoder count is " + msg.lbw_count)
    rospy.loginfo("Right-Front-Wheel turning: " + msg.rfw_dir + "| Encoder count is " + msg.rfw_count)
    rospy.loginfo("Right-Back-Wheel turning: " + msg.rbw_dir + "| Encoder count is " + msg.rbw_count)


if __name__ == '__main__':
    rospy.init_node('mecanum_wheel_subscriber')
    sub = rospy.Subscriber("/wheel_positon", Encoder, callback_mecanum_wheel_data)
    rospy.spin()

'''
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

#following variables are used in order to look for changes in the
last_lin_x=0
last_lin_y=0
last_lin_z=0
last_ang_x=0
last_ang_y=0
last_ang_z=0
changed_dir

def callback_check_changes(msg):
    if

def callback_mecanum_wheel_data(msg):
    rospy.loginfo("Encoder data received: ")
    rospy.loginfo("| Encoder count is " + msg.data[0])
    rospy.loginfo("| Encoder count is " + msg.data[1])
    rospy.loginfo("| Encoder count is " + msg.data[2])
    rospy.loginfo("| Encoder count is " + msg.data[3])
'''