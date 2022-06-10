#! /usr/bin/env python3

import rospy
import numpy as np
from scipy import zeros, signal, random
from sensor_msgs.msg import LaserScan


def laser_callback(msg):

	global new_laser_msg, last_measurements
	
	new_laser_msg.header = msg.header
	new_laser_msg.angle_min = msg.angle_min
	new_laser_msg.angle_max = msg.angle_max
	new_laser_msg.angle_increment = msg.angle_increment
	new_laser_msg.time_increment = msg.time_increment
	new_laser_msg.scan_time = msg.scan_time
	new_laser_msg.range_min = msg.range_min
	new_laser_msg.range_max = msg.range_max
	new_laser_msg.intensities = msg.intensities
	new_laser_msg.ranges = []

	if(last_measurements[0] != None):

		last_measurements.pop()
		last_measurements.insert(0, msg.ranges)


		for i in range(len(msg.ranges)):
						
			sum = 0
			factor = 0

			for a in range(window_size):
				if(abs(last_measurements[0][i] - last_measurements[a][i]) <= max_allowed_difference ):
					sum += last_measurements[a][i]
					factor += 1

			if(factor ==  0):
				new_laser_msg.ranges.append(float('inf'))
			else:
				new_laser_msg.ranges.append(sum/factor)
			
		pub.publish(new_laser_msg)
	
	else:
		last_scan_msg_size = msg.ranges
		for l in range(window_size):
			last_measurements[l] = last_scan_msg_size
	

if __name__ == '__main__':
	
	rospy.init_node('lidar_low_pass')

	node_name = rospy.get_name()
	window_size = rospy.get_param(node_name + "/window_size", 3)
	max_allowed_difference = rospy.get_param(node_name + "/max_allowed_difference", 0.1)
	laser_topic_to_publish = rospy.get_param(node_name + "/laser_topic_to_publish", "/scan_wcr_averaged")
	laser_topic_to_subscribe = rospy.get_param(node_name + "/laser_topic_to_subscribe", "/scan_wcr_ranged")
	
	global new_laser_msg, last_measurements

	new_laser_msg = LaserScan()

	last_measurements = [None] * window_size

	last_timestamp = rospy.Time.now().to_sec()


	sub = rospy.Subscriber(laser_topic_to_subscribe, LaserScan, laser_callback)
	pub = rospy.Publisher(laser_topic_to_publish, LaserScan, queue_size=50)
	rospy.spin()