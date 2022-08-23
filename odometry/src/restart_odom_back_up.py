#!/usr/bin/env python3

#### IMPORTS ####

import rospy
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped

#___________________________________________________________________________________________________


def start_launchfile():
	package = "odometry"
	launch_file = "start_only_hector_for_odom.launch"

	command = "roslaunch  {0} {1}".format(package, launch_file)

	p = subprocess.Popen(command, shell=True)

	state = p.poll()
	if state is None:
		rospy.loginfo("process is running fine")
	elif state < 0:
		rospy.loginfo("Process terminated with error")
	elif state > 0:
		rospy.loginfo("Process terminated without error")

#### CALLBACK-FUNCTION ####
	

def callback_hector_pose(hec_pose):
	global last_time_call, last_pose
	if (	
		(hec_pose.pose.pose.position.x - last_pose.pose.pose.position.x == 0.0) and
		(hec_pose.pose.pose.position.y - last_pose.pose.pose.position.y == 0.0) and
		(hec_pose.pose.pose.orientation.x - last_pose.pose.pose.orientation.x == 0.0) and
		(hec_pose.pose.pose.orientation.y - last_pose.pose.pose.orientation.y == 0.0) and
		(hec_pose.pose.pose.orientation.z - last_pose.pose.pose.orientation.z == 0.0) and
		(hec_pose.pose.pose.orientation.w - last_pose.pose.pose.orientation.w == 0.0) 
	):
		print("Hector SLAM gave exact same pose!")
		pass
	else:
		print("New pose from hector") #delete later
		last_time_call =  (rospy.Time.now()).to_sec()
		last_pose = hec_pose


def timer_callback(event):
	global last_time_call, relaunch_odom_bu

	if (((rospy.Time.now()).to_sec() - last_time_call) >= max_msg_receive_time) and (last_time_call != 0):
		
		start_launchfile()

		last_time_call = (rospy.Time.now()).to_sec()
	else:
		pass

#___________________________________________________________________________________________________

#### MAIN-CODE ####

if __name__ == '__main__':
	#initiate ROS node
	rospy.init_node("look_if_odom_hector_back_up_down")
	global last_pose, last_time_call
	last_time_call = 0
	last_pose = PoseWithCovarianceStamped()
	last_pose.pose.pose.position.x = 0
	last_pose.pose.pose.position.y = 0
	last_pose.pose.pose.orientation.x = 0
	last_pose.pose.pose.orientation.y = 0
	last_pose.pose.pose.orientation.z = 0
	last_pose.pose.pose.orientation.w = 0

	#get parameters from rosparam server
	node_name = rospy.get_name()
	parent_frame = rospy.get_param(node_name + "/parent_frame", "odom")
	child_frame = rospy.get_param(node_name + "/child_frame", "base_link")
	max_msg_receive_time = rospy.get_param(node_name + "/max_msg_receive_time", 3.0)

	#subscribe to hector poseupdate
	rospy.Subscriber("poseupdate", PoseWithCovarianceStamped, callback_hector_pose, queue_size = 1)

	#set up timer to see if new encoder messages arrived
	timer_odom = rospy.Timer(rospy.Duration((max_msg_receive_time/3)), timer_callback)

	rospy.spin()
	timer_odom.shutdown()
                             
#___________________________________________________________________________________________________
      
        