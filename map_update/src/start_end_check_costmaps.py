#! /usr/bin/env python3

import csv
import rospy
import rospkg
import subprocess
import os
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from actionlib_msgs.msg import GoalStatusArray
from pathlib import Path
import pandas as pd
from datetime import date
import glob

#### REMEMBER: LATER THIS CODE MIGHT GET EXTENDED TO SEARCH FOR A RIGHT LOCALIZATION
#### OF THE ROBOT AFTER IT SENDS A SMALL ENOUGH COVARIANCE!!!

#### CODE FOR STARTING THE COSTMAPS USED FOR THE MAPUPDATE, IT ALSO STOPS OR RESTARTS THE COSTMAPS
#### IF FOR EXAMPLE THE ROBOT LOSES ITS POSITION


#### THE FOLLOWING ARE THE USED CALLBACK FUNCTIONS FOR THIS PROGRAM AND ALL SUBSCRIBERS

def callback_amcl(msg):
	global is_localized, covariance_max, last_is_localized, proz_opened, localization_checked, last_localization_checked

	covariance = np.amax(msg.pose.covariance)
	#print("got amcl_pose msg")
	print("covariance ", covariance)

	if covariance < covariance_max :
		is_localized = True
	else:
		is_localized = False

	print("is_localized: ", is_localized)
	print("last_is_localized: ", last_is_localized)
	print("localization_checked: ", localization_checked)
	print("last_localization_checked: ", last_localization_checked)


	if (is_localized and not proz_opened and localization_checked and last_localization_checked):
		start_launchfile()
		rospy.sleep(1.5)				#give some time to start launchfile		
	
	if(((not is_localized) and last_is_localized and proz_opened) or (last_localization_checked and (not localization_checked) and proz_opened)):		#robot was localized right but isn't now/falling flag :: Checking cov as well as localiuation_checked topic for safety
		end_launchfile()
		rospy.sleep(1.0)				#give two seconds for the costmaps to shut down
	
	
	last_is_localized = is_localized
	last_localization_checked = localization_checked
	
def callback_check_right_localization(loc_msg):
	global localization_checked

	if(loc_msg.data):
		localization_checked = True
	else:
		localization_checked = False

def callback_goal_status(goal_msg):
	global last_goal_status, is_localized, proz_opened, localization_checked, last_localization_checked
	
	if (len(goal_msg.status_list) == 0):
		pass
	else:
		if(
			((goal_msg.status_list[0].status == 0 and last_goal_status == 1) or 	#if move_base goes from active drive to searching for a place or
			(goal_msg.status_list[0].status == 3 and last_goal_status == 1)) and	#if move_base says we arrived at our destination and
			proz_opened								#the prozess for the costmaps is running then do...							
		):
			#delete all old costmaps from the last drive to a destination
			folder_files = glob.glob(costmap_destination_dir + "/*")
			for files in folder_files:
				os.remove(files)

			#safe new costmaps
			costmap_c = ("costmap_clearing")
			costmap_nc = ("costmap_no_clearing")
			unoccupied_costmap_c = ("unoccupied_costmap_clearing")
			unoccupied_costmap_nc = ("unoccupied_costmap_no_clearing")

			if((not search_file(costmap_c)) or (not search_file(costmap_nc)) or (not search_file(unoccupied_costmap_c)) or (not search_file(unoccupied_costmap_nc))):
				command_costmap_c = "rosrun  map_server map_saver -f {0} map:={1}".format(costmap_c, topic_for_costmap_clearing)
				command_costmap_nc = "rosrun  map_server map_saver -f {0} map:={1}".format(costmap_nc, topic_for_costmap_no_clearing)
				command_unoccupied_costmap_c = "rosrun  map_server map_saver -f {0} map:={1}".format(unoccupied_costmap_c, topic_for_unoccupied_costmap_clearing)
				command_unoccupied_costmap_nc = "rosrun  map_server map_saver -f {0} map:={1}".format(unoccupied_costmap_nc, topic_for_unoccupied_costmap_no_clearing)

				commands = [command_costmap_c, command_costmap_nc, command_unoccupied_costmap_c, command_unoccupied_costmap_nc]	#create a list containing all maps

				for command in commands:
					proz_map = subprocess.Popen(command, shell=True, cwd=costmap_destination_dir)

					state = proz_map.poll()
					if state is None:
						rospy.loginfo("process is running fine. Saving map!")
					elif state < 0:
						rospy.loginfo("Process terminated with error. ERROR MAP COULD NOT BEEN SAFED!!!")
					elif state > 0:
						rospy.loginfo("Process terminated without error. Map saved!")

					rospy.sleep(0.5);
				
				rospy.sleep(1.5)		#give a little time for all maps to be stored
				

			if((search_file(costmap_c)) and (search_file(costmap_nc)) and (search_file(unoccupied_costmap_c)) and (search_file(unoccupied_costmap_nc))):

				command = "rosrun  {0} {1}".format("map_update", "map_update_on_destination_drive")

				proz_mapupdate = subprocess.Popen(command, shell=True)

				state = proz_mapupdate.poll()
				if state is None:
					rospy.loginfo("process is running fine.")
				elif state < 0:
					rospy.loginfo("Process terminated with error.")
				elif state > 0:
					rospy.loginfo("Process terminated without error.")

				rospy.sleep(0.5);	#time to process

				end_launchfile();
				
				rospy.sleep(0.7);	#time to end/kill file

				if (is_localized and not proz_opened and localization_checked and last_localization_checked):
					start_launchfile()
					rospy.sleep(0.5)	#time to start file
		
		last_goal_status = goal_msg.status_list[0].status


#### FOLLOWING ARE THE USED FUNCTIONS/METHODS FOR THIS PROGRAM

def start_launchfile():			#start costmap launchfile
	print("starting launfile")

	command = "roslaunch  {0} {1}".format(package_with_costmap_launchfile, mapupdate_costmap_launchfile)

	global proz, proz_opened

	if(not proz_opened):
		proz = subprocess.Popen(command, shell=True)
		proz_opened = True

		state = proz.poll()
		if state is None:
			rospy.loginfo("process is running fine")
		elif state < 0:
			rospy.loginfo("Process terminated with error")
			proz_opened = False
		elif state > 0:
			rospy.loginfo("Process terminated without error")
			proz_opened = False

def end_launchfile():			#end costmap launchfile
	print("ending launchfile")
	
	global proz, proz_opened

	if(proz_opened):		#check if costmap launchfile has already been launched

		proz.terminate()
		rospy.sleep(2.0)	#give time to closs process
		proz.kill()
		proz_opened = False

		state = proz.poll()
		if state is None:
			rospy.loginfo("process is running fine, but should been terminated!")
			proz_opened = True
		elif state < 0:
			rospy.loginfo("Process terminated with error")
		elif state > 0:
			rospy.loginfo("Process terminated without error")	

def search_file(filename):
	
	filename_whole = (filename + ".pgm")

	for root, dir, files in os.walk(costmap_destination_dir):
		if filename_whole in files:
			return True	#if the searched file is found return True

	return False

def directory_check_and_generate(path):
	if(not (os.path.isdir(path))):
		os.mkdir(path)
		print("Created directory ", path, " because it was not present! (Is needed for filehandling!)")

def create_unoccupied_base_map():

	command = "rosrun  {0} {1}".format("map_update", "generate_unoccupied_inital_map")

	proz_mapupdate = subprocess.Popen(command, shell=True)

	state = proz_mapupdate.poll()
	if state is None:
		rospy.loginfo("process is running fine.")
	elif state < 0:
		rospy.loginfo("Process terminated with error.")
	elif state > 0:
		rospy.loginfo("Process terminated without error.")

	rospy.sleep(0.2);			#give some time for the map to be safed!

#### FOLLOWING THE MAIN CODE

if __name__ == '__main__':

	#initate node
	rospy.init_node('start_end_check_mapupdate_costmaps')

	#global variable which value can be changed through rosparam
	global covariance_max, is_localized, proz, proz_opened, last_is_localized, costmap_destination_dir, same_day_maps_dir, different_day_maps_dir, updated_map_dir
	global localization_checked, last_localization_checked, last_goal_status
	#global date_match	#boolean variable to see if the date between a csv file (which safes the last day this program was running)
				#and today do match (so if it is the same day)

	#get the name of the node if started from a .launch file
	node_name = rospy.get_name()

	#get params started from a .launch file
	covariance_max = rospy.get_param(node_name + "/covariance_max", 0.30)
	package_with_costmap_launchfile = rospy.get_param(node_name + "/package_with_costmap_launchfile", "map_update")
	mapupdate_costmap_launchfile = rospy.get_param(node_name + "/mapupdate_costmap_launchfile", "mapupdate_costmap.launch")
	robot_localization_checked_topic = rospy.get_param(node_name + "/robot_localization_checked_topic", "/robot_localization_is_checked")
	topic_for_costmap_clearing = rospy.get_param(node_name + "/topic_for_costmap_clearing", "/update_costmap_clearing/costmap/costmap")
	topic_for_costmap_no_clearing = rospy.get_param(node_name + "/topic_for_costmap_no_clearing", "/update_costmap_no_clearing/costmap/costmap")
	topic_for_unoccupied_costmap_clearing = rospy.get_param(node_name + "/topic_for_unoccupied_costmap_clearing", "/update_unoccupied_clearing/costmap/costmap")
	topic_for_unoccupied_costmap_no_clearing = rospy.get_param(node_name + "/topic_for_unoccupied_costmap_no_clearing", "/update_unoccupied_no_clearing/costmap/costmap")

	#define variables which are used for file-handling
	costmap_destination_dir = "/build_costmaps_on_destination"
	same_day_maps_dir = "/same_day_maps"
	different_day_maps_dir = "/different_day_maps"
	updated_map_dir = "/updated_map_after_multiple_days"
	csv_file_dir = "/csv_files"
	unoccupied_bas_map_dir = "/unoccupied_base_map"


	#get full paths to directories with the help of RosPack
	rospack = rospkg.RosPack()
	mapupdate_path = rospack.get_path('map_update')
	print(mapupdate_path)#delete
	same_day_maps_dir = mapupdate_path + same_day_maps_dir
	different_day_maps_dir = mapupdate_path + different_day_maps_dir
	updated_map_dir = mapupdate_path + updated_map_dir
	costmap_destination_dir = mapupdate_path + costmap_destination_dir
	csv_file_dir = mapupdate_path + csv_file_dir
	unoccupied_bas_map_dir = mapupdate_path + unoccupied_bas_map_dir

	#Now we check if the needed directories for handling all map files exist. If they do not exist we create them!
	directory_check_and_generate(csv_file_dir)
	directory_check_and_generate(unoccupied_bas_map_dir)
	directory_check_and_generate(same_day_maps_dir)
	directory_check_and_generate(different_day_maps_dir)
	directory_check_and_generate(updated_map_dir)
	directory_check_and_generate(costmap_destination_dir)

	#Now we use the classes made in c++ to generate a unoccupied base map (which will be used for the unoccupied costmaps)
	#this will only be needed once when we start this program!
	create_unoccupied_base_map();
	
	#define  subscriber to the amcl pose to see if the covariance is under a certain threshold
	sub_amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_amcl)
	#define a subscriber to the topic which tells us if the localization of the robot is correct (sent 'check_right_localization.py' programm)
	sub_right_loc = rospy.Subscriber(robot_localization_checked_topic, Bool, callback_check_right_localization)
	#define a subscriber to GoalStatusArray msg ("move_base/status topic") to see if the robot arrived at a destination (falling edge on goal status array)
	sub_goal_status = rospy.Subscriber("/move_base/status", GoalStatusArray, callback_goal_status)

	#initialize global variables
	is_localized = False
	last_is_localized = False
	proz_opened = False
	localization_checked= False
	last_localization_checked = False
	last_goal_status = 0

	rospy.spin()