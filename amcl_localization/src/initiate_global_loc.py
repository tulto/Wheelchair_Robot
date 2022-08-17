#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty

#definde callbak
def callback_localization_checker(msg):
	
	global loc_number, last_loc_status, was_cor_loc

	if(not msg.data and last_loc_status):
		was_cor_loc = True
		last_loc_status = False

	if(not msg.data and was_cor_loc):
		if(loc_number < max_loc_lost_number):
			loc_number += 1

	if(loc_number == max_loc_lost_number):
		was_cor_loc = False
		loc_number = 0
		rospy.wait_for_service("global_localization", timeout=2.0)
		try:
			global_loc_srv = rospy.ServiceProxy('global_localization', Empty)
			call_global_loc = global_loc_srv()
			
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	if(msg.data):
		loc_number = 0
		was_cor_loc = False

	last_loc_status = msg.data


#write main
if __name__ == '__main__':

	#initate node
	rospy.init_node('reinitialize_global_localization')

	#global variable which value can be changed through rosparam
	global loc_number, max_loc_lost_number, last_loc_status, was_cor_loc
	was_cor_loc = False
	last_loc_status = False
	loc_number = 0

	#get the name of the node if started from a .launch file
	node_name = rospy.get_name()
	#get params started from a .launch file
	max_loc_lost_number = rospy.get_param(node_name + "/max_loc_lost_number", 3)

	#create subscriber for the localization checker "robot_localization_is_checked"
	loc_sub = rospy.Subscriber("robot_localization_is_checked", Bool, callback_localization_checker)

	rospy.spin()