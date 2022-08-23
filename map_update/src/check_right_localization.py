#! /usr/bin/env python3

from operator import le
from re import X
from turtle import left
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point32
from sensor_msgs.msg import LaserScan, PointCloud
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

#### REMEMBER: LATER THIS CODE MIGHT GET EXTENDED TO SEARCH FOR A RIGHT LOCALIZATION
#### OF THE ROBOT AFTER IT SENDS A SMALL ENOUGH COVARIANCE!!!

#### CODE FOR STARTING THE COSTMAPS USED FOR THE MAPUPDATE, IT ALSO STOPS OR RESTARTS THE COSTMAPS
#### IF FOR EXAMPLE THE ROBOT LOSES ITS POSITION!!!

class CheckLocalization:
	def __init__(self, cov_max, min_localized_poses_in_row, minimal_match, minimal_used_beam_endpoints):
		self.max_covariance = cov_max
		self.min_localized_poses_row = min_localized_poses_in_row
		self.min_match = minimal_match
		self.is_localized = False
		self.last_localized = False
		self.localized_for_iter = 1
		#self.trans = None
		#self.rot = None
		self.map_received = False
		self.scan_received = False
		self.ready_to_receive_scan = False
		self.ready_to_receive_map = False
		self.loaded_scan = LaserScan()
		self.robot_pose = Pose()
		self.loaded_map = OccupancyGrid()
		self.sum_corr = 0
		self.usable_endpoints = 0
		self.localization_checked_and_right = False
		self.min_used_endpoints = minimal_used_beam_endpoints

	def check_covariance(self, amcl_msg):

		covariance = np.amax(amcl_msg.pose.covariance)

		#print("covariance: ", covariance)

		if covariance < self.max_covariance:
			if(self.localized_for_iter < self.min_localized_poses_row):
				self.localized_for_iter += 1

		else:
			self.localized_for_iter = 1			#reset counter for how much pose messages have a "localized" covariance / covariance smaller than max covariance

		if(self.localized_for_iter == self.min_localized_poses_row):
			self.is_localized = True
		else:
			self.is_localized = False

		#print("how_long_localized: ", self.localized_for_iter)
		#print("is_localized: ", self.is_localized)

		if(self.is_localized and not self.last_localized):	#if rising edge on is_localized
			self.ready_to_receive_scan  = True
			self.ready_to_receive_map  = True
			self.robot_pose = amcl_msg.pose.pose		#give Amcl pose to robot_pose for further use

		if(not self.is_localized and self.last_localized):	#if falling edge on is_localized
			self.ready_to_receive_scan  = False
			self.ready_to_receive_map  = False
			self.scan_received = False
			self.localized_for_iter = 1
			self.sum_corr = 0.0
			self.usable_endpoints = 0
			self.loaded_scan = LaserScan()			#reset loaded_scan
			self.robot_pose = Pose()			#reset Pose
			self.reset_correct_localization()
			
		self.last_localized = self.is_localized			#give is_localized to last_localized in order to search for rising or falling flags

	def get_map(self, map_msg):
		print("Getting_map")
		if(not self.map_received):
			self.loaded_map = map_msg
			self.map_received = True
			self.ready_to_receive_map = False
			print("Got map")
	
	def get_laserscan(self, scan_msg):
		if(self.is_localized and self.ready_to_receive_scan and not self.scan_received):
			self.loaded_scan = scan_msg
			self.scan_received = True
			self.ready_to_receive_scan = False
			print("Got scan")

	def check_robot_localization(self):

		#for the calculation of the coordinate of the beam_endpoints we need to know that the LaserScan msg 
		#ranges start at index 0 with the angle 0 and and end at index max_index with 360 degree (increasing with angle_increment)
		#FURTHER: We only need the x and y as well as the theta coordinate because we use a 2 dimensional map
		if(self.is_localized and self.scan_received and self.map_received and not self.localization_checked_and_right):
			quaternion_from_pose = (
				self.robot_pose.orientation.x,
				self.robot_pose.orientation.y,
				self.robot_pose.orientation.z,
				self.robot_pose.orientation.w,
			)
			euler_pose = tf.transformations.euler_from_quaternion(quaternion_from_pose)	#get euler orientation for calculating the x and y coordinates of beam_endpoints
			
			coord_beam_endpoints = []
			
			#print("robot_pose: ", math.floor((abs(self.loaded_map.info.origin.position.x) + self.robot_pose.position.x)/self.loaded_map.info.resolution), " , ",  math.floor((abs(self.loaded_map.info.origin.position.y) + self.robot_pose.position.y)/self.loaded_map.info.resolution))
			#print("map orig y in pix: ", (self.loaded_map.info.origin.position.y)/self.loaded_map.info.resolution)

			global vis_pc #DELETE
			vis_pc.points = []

			for dist in range(len(self.loaded_scan.ranges)):
				#following we calculate the x and y coordinates of the beam endpoints form the laserScan msg
				#relative to the lower left end of the loaded map / given map file (pgm map file)
				if(self.loaded_scan.ranges[dist] != float('inf')):
					coord_single_endpoint_x = (abs(self.loaded_map.info.origin.position.x) + self.robot_pose.position.x + (math.cos(euler_pose[2] + math.pi + (dist*self.loaded_scan.angle_increment))*self.loaded_scan.ranges[dist])) #add pi (or 180°) to the rotation because the lidar sensor starts measurement at the opposite side of the lidar x-axis
					coord_single_endpoint_y = (abs(self.loaded_map.info.origin.position.y) + self.robot_pose.position.y + (math.sin(euler_pose[2] + math.pi + (dist*self.loaded_scan.angle_increment))*self.loaded_scan.ranges[dist])) #so it starts 180° later, without this the lidar points would be turned 180° corresponding to the scanned environment (map)

					#DELETE LATER
					point_laser = Point32()
					point_laser.x = (self.robot_pose.position.x + (math.cos(euler_pose[2] + math.pi + (dist*self.loaded_scan.angle_increment))*self.loaded_scan.ranges[dist]))
					point_laser.y = (self.robot_pose.position.y + (math.sin(euler_pose[2] + math.pi + (dist*self.loaded_scan.angle_increment))*self.loaded_scan.ranges[dist]))
					point_laser.z = 0.05
					vis_pc.header.stamp = rospy.Time.now()
					vis_pc.points.append(point_laser)


					coord_beam_endpoints.append([coord_single_endpoint_x, coord_single_endpoint_y])
				else:
					coord_beam_endpoints.append([None, None])

			#REMINDER: In the OccupancyGird message the occupancy grid map values start at the lower left side of the map and 
			#	   build up to the upper right side so index 0 is lower left pixel and index index_max is upper right pixel
			#	   from left to right from bottom to top
			#	   Also the position in x is in width and y in height
			
			#print("resolution: ", self.loaded_map.info.resolution)
			#print("width: ", self.loaded_map.info.width)
			#print("height: ", self.loaded_map.info.height)
			
			#NOTE: THE FOLLOWING WILL NOT BE DELETED!! TO REMIND THE PROGRAMMER THAT 
			#IN PYHTON THIS IS POSSIBLE BUT FALSE BECAUSE EVERY ROW WILL HAVE THE SAME
			#VALUES THE INNER LIST HAS!!!!
			#map_2d = [[None]*self.loaded_map.info.width]*self.loaded_map.info.height
			
			map_2d = []

			for y_coord in range(self.loaded_map.info.height):
				map_row = []
				for x_coord in range(self.loaded_map.info.width):
					map_row.append(self.loaded_map.data[(y_coord*self.loaded_map.info.width) + x_coord])	#safe map_data in a list which will later be appended to two dimensional matrix
				map_2d.append(map_row)

			self.sum_corr = 0
			self.usable_endpoints = 0

			for measurement in range(len(self.loaded_scan.ranges)):
				
				if(
					coord_beam_endpoints[measurement][0] != None and
					coord_beam_endpoints[measurement][1] != None and
					len(map_2d) > 0
				):
					map_pixel_x = abs(math.floor((coord_beam_endpoints[measurement][0])/self.loaded_map.info.resolution))
					map_pixel_y = abs(math.floor((coord_beam_endpoints[measurement][1])/self.loaded_map.info.resolution))
										
					if(
						map_pixel_x >= 0 and 
						map_pixel_x < len(map_2d[0]) and 
						map_pixel_y >= 0 and 
						map_pixel_y < len(map_2d)
					):
						left = False
						right = False
						upper = False
						lower = False
						
						if(map_pixel_y-1 >= 0):
							lower = True
						if(map_pixel_y+1 < self.loaded_map.info.height):
							upper = True
						if(map_pixel_x-1 >= 0):
							left = True
						if(map_pixel_x+1 < self.loaded_map.info.width):
							right = True
						
						if(map_2d[map_pixel_y][map_pixel_x] == 100):
							self.sum_corr += 1.0				#if we have a occupied cell in the sent map at the same cell where 
													#we have a beam endpoint we add hole point (1.0) to the correlation sum sum_corr
						elif(left and (map_2d[map_pixel_y][map_pixel_x-1] == 100)):
							self.sum_corr += 0.5			#if we have a occupied cell near the beam endpoint cell we (so one cell left,
												#right, up or down) we add 0.5 to the correlation sum value
						elif(right and (map_2d[map_pixel_y][map_pixel_x+1] == 100)):
							self.sum_corr += 0.5			#if we have a occupied cell near the beam endpoint cell we (so one cell left,
												#right, up or down) we add 0.5 to the correlation sum value
						elif(upper and (map_2d[map_pixel_y+1][map_pixel_x] == 100)):
							self.sum_corr += 0.5			#if we have a occupied cell near the beam endpoint cell we (so one cell left,
												#right, up or down) we add 0.5 to the correlation sum value
						elif(lower and (map_2d[map_pixel_y-1][map_pixel_x] == 100)):
							self.sum_corr += 0.5			#if we have a occupied cell near the beam endpoint cell we (so one cell left,
												#right, up or down) we add 0.5 to the correlation sum value
						
						self.usable_endpoints += 1

		
		laser_point_pub.publish(vis_pc)
		if(self.usable_endpoints == 0):
			self.localization_checked_and_right = False
		else:
			#print("sum_corr: ", self.sum_corr)
			#print("usable_endpoints: ", self.usable_endpoints)
			#print("match_factor: ", (self.sum_corr/self.usable_endpoints))	
			if(((self.sum_corr/self.usable_endpoints) >= self.min_match) and (self.usable_endpoints >= self.min_used_endpoints)):
				self.localization_checked_and_right = True
			else:
				self.reset_correct_localization()
		
	def robot_correctly_localized(self):
		return (self.localization_checked_and_right)

	def reset_correct_localization(self):
		self.localization_checked_and_right = False
		self.ready_to_receive_scan  = False
		self.ready_to_receive_map  = False
		self.scan_received = False
		self.localized_for_iter = 1			#reset counter for right loclization
		self.sum_corr = 0.0				#reset the sum_corr
		self.usable_endpoints = 0			#reset usable_endpoints
		self.loaded_scan = LaserScan()			#reset loaded_scan
		self.robot_pose = Pose()			#reset Pose


def callback_amcl(msg):
	global localization_checker, robo_loc_checked

	localization_checker.check_covariance(msg)

	robo_loc_checked.data = localization_checker.robot_correctly_localized()

	loc_pub.publish(robo_loc_checked)
	
		
def callback_laserscan(laser_msg):
	global localization_checker
	
	localization_checker.get_laserscan(laser_msg)
	localization_checker.check_robot_localization()

def callback_map(map_msg):
	global localization_checker

	localization_checker.get_map(map_msg)
		

if __name__ == '__main__':

	#initate node
	rospy.init_node('right_localization_check')

	#global variable which value can be changed through rosparam
	global localization_checker, robo_loc_checked, vis_pc

	#get the name of the node if started from a .launch file
	node_name = rospy.get_name()
	#get params started from a .launch file
	covariance_max = rospy.get_param(node_name + "/covariance_max", 0.22)
	#start_frame = rospy.get_param(node_name + "/start_frame", "/map")
	#end_frame = rospy.get_param(node_name + "/end_frame", "/base_link")
	min_localized_amcl_poses_in_row = rospy.get_param(node_name + "/min_localized_amcl_poses_in_row", 5)
	laserscan_topic_to_subscribe = rospy.get_param(node_name + "/laserscan_topic_to_subscribe", "/scan_wcr_averaged")
	map_topic_to_subscribe = rospy.get_param(node_name + "/map_topic_to_subscribe", "/map")
	topic_name_is_right_localized = rospy.get_param(node_name + "/topic_name_is_right_localized", "/robot_localization_is_checked")
	min_used_beam_endpoints_for_localization_check = rospy.get_param(node_name + "/min_used_beam_endpoints_for_localization_check", 65)
	min_match_percentage = rospy.get_param(node_name + "/min_match_percentage", 0.7)
	
	#give initial values to parameters
	localization_checker = CheckLocalization(covariance_max, min_localized_amcl_poses_in_row, min_match_percentage, min_used_beam_endpoints_for_localization_check)
	robo_loc_checked = Bool()
	robo_loc_checked.data = False
	vis_pc = PointCloud()
	vis_pc.header.frame_id = "map"

	#publisher vor checking delete later
	laser_point_pub = rospy.Publisher("/laser_points", PointCloud, queue_size=10)

	#create publisher wich send a boolean value stating if the robot has been localized or not
	loc_pub = rospy.Publisher(topic_name_is_right_localized, Bool, queue_size=5)
	
	#define subscriber to the amcl pose to see if the covariance is under a certain threshold
	sub_amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_amcl)
	
	#definde subscriber to LaserScan msg from the laser scan used for generating map as well as for localization
	laser_sub = rospy.Subscriber(laserscan_topic_to_subscribe, LaserScan, callback_laserscan)

	#definde subscriber to the used map for localization with amcl 
	map_sub = rospy.Subscriber(map_topic_to_subscribe, OccupancyGrid, callback_map)

	#generate a tf listener
	tf_listener = tf.TransformListener()
	
	rospy.spin()

		
