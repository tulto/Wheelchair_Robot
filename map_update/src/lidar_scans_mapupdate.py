#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np


#### THIS CODE IS USED TO ONLY GENERATE (OR BETTER SEND) LIDAR SCAN DATA TO THE 
#### COSTMAPS USED FOR THE MAPUPDATE IF THE COVARIANCE OF AMCL IS UNDER A CERTAIN
#### THRESHOULD DEFINED BY THE AMCL_STATUS MESSAGE

def callback_amcl(msg):
	global is_localized, covariance_max

	covariance = np.amax(msg.pose.covariance)

	if covariance < covariance_max :
		is_localized = True
	else:
		is_localized = False

def callback(msg):
	global is_localized, laser_msg_localized

	laser_msg_localized = msg;

	if(is_localized):
		pub.publish(laser_msg_localized)
	

if __name__ == '__main__':

	#initate node
	rospy.init_node('lidar_scans_check_for_mapupdate')

	#global variable which value can be changed through rosparam
	global covariance_max

	#get the name of the node if started from a .launch file
	node_name = rospy.get_name()
	#get params started from a .launch file
	laser_scan_topic_subscribe = rospy.get_param(node_name + "/laser_scan_topic_subscribe", "/scan_wcr_averaged")
	laser_scan_topic_publish = rospy.get_param(node_name + "/laser_scan_topic_publish", "/scan_wcr_mapupdate_avg")
	covariance_max = rospy.get_param(node_name + "/covariance_max", 0.18)
	#define publisher for new laser_scan_msg
	pub = rospy.Publisher(laser_scan_topic_publish, LaserScan, queue_size=2)
	#define subscriber from original laser_scan_msg
	sub = rospy.Subscriber(laser_scan_topic_subscribe, LaserScan, callback)
	#define  subscriber to the amcl pose to see if the covariance is under a certain threshold
	sub_amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_amcl)

	#define global used parameter
	global is_localized, laser_msg_localized

	is_localized = False
	laser_msg_localized = LaserScan()
	

	rospy.spin()