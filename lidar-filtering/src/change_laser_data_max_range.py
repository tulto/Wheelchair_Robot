#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    laser_msg = msg;
    laser_msg.range_max = new_max_range

    pub.publish(laser_msg)

if __name__ == '__main__':

	node_name = rospy.get_name()
	laser_scan_topic_subscribe = rospy.get_param(node_name + "/laser_scan_topic_subscribe", "/scan_wcr")
	laser_scan_topic_publish = rospy.get_param(node_name + "/laser_scan_topic_publish", "/scan_wcr_changed_max_range")
	new_max_range = rospy.get_param(node_name + "new_max_range", 2000)
	rospy.init_node('scan_max_range_changer')

	pub = rospy.Publisher(laser_scan_topic_publish, LaserScan, queue_size=25)

	sub = rospy.Subscriber(laser_scan_topic_subscribe, LaserScan, callback)

	laser_msg = LaserScan();

	rospy.spin()