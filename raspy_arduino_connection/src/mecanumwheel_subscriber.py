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

