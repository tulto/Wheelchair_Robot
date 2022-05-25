#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist


def callback_is_autonomous_nav_active(msg):
	global last_status
	if(len(msg.status_list) != 0):
		if(last_status == 1 and msg.status_list[0].status == 3):
			
			stop_msg.stamp.nsecs = 0
			stop_msg.stamp.secs = 0
			stop_msg.id = ""

			for i in range(2):
				stop_pub.publish(stop_msg)
				rospy.sleep(0.05)
			
			cmd_vel_zero_msg.linear.x = 0
			cmd_vel_zero_msg.linear.y = 0
			cmd_vel_zero_msg.linear.z = 0
			cmd_vel_zero_msg.angular.x = 0
			cmd_vel_zero_msg.angular.y = 0
			cmd_vel_zero_msg.angular.z = 0
			for i in range(10):
				pub_cmd_vel.publish(cmd_vel_zero_msg)
				rospy.sleep(0.05)
		
		last_status = msg.status_list[0].status #give status to last_status


if __name__ == '__main__':
	rospy.init_node('reset_cmd_vel_node', anonymous=False)
	
	global last_status
	last_status = 0

	sub_mb = rospy.Subscriber("/move_base/status", GoalStatusArray, callback_is_autonomous_nav_active) #subscriber for the current navigation status
	stop_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=5) #publisher to publish a stop message the the navigation planner
	pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5) #publisher to publish cmd vel 0 messages to the arduino 

	stop_msg = GoalID()		#create a GoalID msg object for publishing the stop message when arriving
	cmd_vel_zero_msg = Twist()	#create a Twist msg for sending to the arduino in order to make sure it stops at its destination
        
	rospy.spin()