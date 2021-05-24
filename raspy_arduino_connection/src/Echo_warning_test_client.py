#!/usr/bin/env python3
import rospy
from services_and_messages.srv import WarningEchoTest

if __name__=='__main__':
    rospy.init_node("execute_echo_warning", anonymous=True)
    rospy.wait_for_service("/echo_test_warning")

    try:
        warning_request = rospy.ServiceProxy("/echo_test_warning", WarningEchoTest)
        response = warning_request("el")
        if response.acknowledge:
            rospy.loginfo("Warning test was successful!")
        else:
            rospy.loginfo("Warning test wasn't successful. \Maybe your sent message was wrong.")
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))