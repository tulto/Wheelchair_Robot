#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Empty
import rospkg
import subprocess
import os


#### DEFINE NEEDED CALLBACK FUNCTIONS

def callback_vizualization_map_building(msg):

	global proc_map_run, proc_viz_run, proc_map, proc_viz, last_viz_map_value

	#on rising edge
	if(msg.data and not last_viz_map_value):

		command_slam = "exec roslaunch  {0} {1}".format(slam_algorithm_launchfile_package, slam_algorithm_launchfile_name)
	
		command_viz = "exec roslaunch  {0} {1}".format("slam_algorithms", "launch_map_visualization.launch")

		if(not proc_viz_run):
			proc_viz = subprocess.Popen(command_viz, shell=True)
			proc_viz_run = True

			state = proc_viz.poll()
			if state is None:
				rospy.loginfo("Process is running fine")
			elif state < 0:
				rospy.loginfo("Process terminated with error")
				proc_viz_run = False
			elif state > 0:
				rospy.loginfo("Process terminated without error")

		rospy.sleep(2.0)	#give some time to start visualization
		
		if(not proc_map_run):
			proc_map = subprocess.Popen(command_slam, shell=True)
			proc_map_run = True

			state = proc_map.poll()
			if state is None:
				rospy.loginfo("Process is running fine")
			elif state < 0:
				rospy.loginfo("Process terminated with error")
				proc_map_run = False
			elif state > 0:
				rospy.loginfo("Process terminated without error")

	#on falling edge
	if(not msg.data and last_viz_map_value):
		
		if(proc_viz_run):
			proc_viz.terminate()
			rospy.sleep(2.5)	#give time to close process
			proc_viz.kill()
			proc_viz_run = False

			state = proc_viz.poll()
			if state is None:
				rospy.loginfo("process is running fine, but should been terminated!")
				proc_viz_run = True
			elif state < 0:
				rospy.loginfo("Process terminated with error")
			elif state > 0:
				rospy.loginfo("Process terminated without error")

		if(proc_map_run):
			proc_map.terminate()
			rospy.sleep(2.5)	#give time to close process
			proc_map.kill()
			proc_map_run = False

			state = proc_map.poll()
			if state is None:
				rospy.loginfo("process is running fine, but should been terminated!")
				proc_map_run = True
			elif state < 0:
				rospy.loginfo("Process terminated with error")
			elif state > 0:
				rospy.loginfo("Process terminated without error")
			

	last_viz_map_value = msg.data

def callback_stop_map_generation(msg):
	
	global proc_map, proc_map_run

	if(proc_map_run):
			proc_map.terminate()
			rospy.sleep(2.5)	#give time to close process
			proc_map.kill()
			rospy.sleep(1.0)	#give time to close process
			proc_map_run = False

			state = proc_map.poll()
			if state is None:
				rospy.loginfo("process is running fine, but should been terminated!")
				proc_map_run = True
			elif state < 0:
				rospy.loginfo("Process terminated with error")
			elif state > 0:
				rospy.loginfo("Process terminated without error")

	

#### DEFINE GENERAL FUNCTIONS WHICH ARE NEEDED
def directory_check_and_generate(path):
	if(not (os.path.isdir(path))):
		os.mkdir(path)
		print("Created directory ", path, " because it was not present! (Is needed for filehandling!)")

#### FOLLOWING THE MAIN CODE

if __name__ == '__main__':

	#initate node
	rospy.init_node('map_builder_start')

	#get the name of the node if started from a .launch file
	node_name = rospy.get_name()

	#get params started from a .launch file
	map_destination_package = rospy.get_param(node_name + "/map_destination_package", "navigation")
	map_destination_folder = rospy.get_param(node_name + "/map_destination_folder", "/maps/test")
	slam_algorithm_launchfile_package = rospy.get_param(node_name + "/slam_algorithm_launchfile_package", "slam_algorithms")
	slam_algorithm_launchfile_name = rospy.get_param(node_name + "/slam_algorithm_launchfile_name", "karto_slam_real_robot.launch")

	#get full paths to directories with the help of RosPack
	rospack = rospkg.RosPack()
	map_destination_path = rospack.get_path(map_destination_package)
	map_destination_path = (map_destination_path +map_destination_folder)

	#checking if needed directory exists, otherwise we generate it
	directory_check_and_generate(map_destination_path)

	#definde global needed variables
	global proc_viz_run		#variable to see if subprocess for visualization is running
	global proc_map_run	#variable to see if subprocess for mapping is running
	
	proc_viz_run = False	#visualiuation process not running
	proc_map_run = False	#mapping process not running

	global last_viz_map_value	#safe last value on to vizualise_map topic for edge detection
	last_viz_map_value = False	#set to false

	#define a subscriber to a boolean msg which tells us if we should start the mapping phase
	sub_right_loc = rospy.Subscriber("vizualise_map", Bool, callback_vizualization_map_building)

	#define a subscriber which tells us if the 
	sub_right_loc = rospy.Subscriber("end_mapping", Empty, callback_stop_map_generation)

	rospy.spin()