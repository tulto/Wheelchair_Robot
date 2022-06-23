#!/usr/bin/env python3

import numpy as np
import rospy
import csv, sys

import tf2_ros
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_srvs.srv import Empty


class NavHandler:
    def __init__(self):
        self.sub_stop = rospy.Subscriber("/nav_cancel", String, self.callback_cancel)
        self.pub_stop = rospy.Publisher("/move_base/cancel", GoalID, queue_size=5)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.sub_goal = rospy.Subscriber("/nav_goal", String, self.callback_goal)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)

        self.header = ['name', 'x', 'y', 'z', 'u_x', 'u_y', 'u_z', 'u_w']
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub_goal = rospy.Subscriber("/nav_save_position", String, self.callback_save_pos) 

        #self.sub_locate = rospy.Subscriber("/nav_locate", String, self.callback_locate) 
        #self.pub_locate = rospy.Publisher("/nav_locate_status", String, queue_size=5)

        self.sub_amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_amcl_pose)
        self.pub_amcl_status = rospy.Publisher("/amcl_status", Bool, queue_size=5)
        self.cov = 100
        self.cov_max = 0.3

    def callback_goal(self, msg):
        """send a string with matching param equivalent to set goal

        Args:
            msg (_type_): std_msgs/String
        """

        self.delete_costmap()
        goal = "/navigation_goal_stop_node/" 
        goal += msg.data

        pose = rospy.get_param(goal)
        
        msg_goal = PoseStamped()
        msg_goal.header.frame_id = "map"
        
        msg_goal.pose.position.x = pose[0]
        msg_goal.pose.position.y = pose[1]
        msg_goal.pose.position.z = pose[2]

        msg_goal.pose.orientation.x = pose[3]
        msg_goal.pose.orientation.y = pose[4]
        msg_goal.pose.orientation.z = pose[5]
        msg_goal.pose.orientation.w = pose[6]
        if self.cov < self.cov_max:
            self.pub_goal.publish(msg_goal)

    def callback_save_pos(self, msg):
        if self.tfBuffer.can_transform('base_link', 'map', rospy.Time(0)):
            trans = self.tfBuffer.lookup_transform('base_link', 'map', rospy.Time(0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            q_x = trans.transform.rotation.x
            q_y = trans.transform.rotation.y
            q_z = trans.transform.rotation.z
            q_w = trans.transform.rotation.w

            data = [msg.data, x, y, 0, 0, 0, q_z, q_w]

            path = '/home/timo/catkin_ws/src/wheelchair_robot/robot_gui/src/positions.csv'

            with open(path, 'r+') as in_file:
                reader = csv.reader(in_file)
                rows = [row for row in csv.reader(in_file) if msg.data not in row]
                in_file.seek(0)
                in_file.truncate()
                writer = csv.writer(in_file)
                writer.writerows(rows)
                #in_file.close()

            with open(path, 'a') as append:
                appender = csv.writer(append)
                appender.writerow(data)
                #append.close()

    """
            lines = list()
            with open(path, 'r') as read_file:
                reader = csv.reader(read_file)
                for row in read_file:
                    if(row[0] != msg.data):
                        lines.append(row)
                read_file.close()

            print(lines)


            with open(path, 'w') as write_file:
                writer = csv.writer(write_file)
                writer.writerows(lines)
                write_file.close()


            with open(path, 'a+') as append:
                appender = csv.writer(append)
                appender.writerow(data)
                append.close()

    """
    """
    def callback_locate(self, msg):
        self.delete_costmap()
        
        msg_goal = PoseStamped()
        msg_goal.header.stamp = rospy.Time()
        msg_goal.header.stamp = rospy.Time()
        msg_goal.header.frame_id = "base_link"
        
        msg_goal.pose.position.x = -0.5
        msg_goal.pose.position.y = 0
        msg_goal.pose.position.z = 0

        msg_goal.pose.orientation.x = 0
        msg_goal.pose.orientation.y = 0
        msg_goal.pose.orientation.z = 0
        msg_goal.pose.orientation.w = 1
        self.pub_goal.publish(msg_goal)
    """

    def callback_cancel(self, msg):
        """simple cancel option of navigation

        Args:
            msg (_type_): std_msgs/String
        """
        msg_stop = GoalID()
        msg_stop.stamp.nsecs = 0
        msg_stop.stamp.secs = 0
        msg_stop.id = ""
        for i in range(10):
            self.pub_stop.publish(msg_stop)
            rospy.sleep(0.05)

        # publish cmd_vel 0
        #time.sleep(100) comment this line because it seems to have a problem with ros
        msg_vel = Twist()
        msg_vel.linear.x = 0
        msg_vel.linear.y = 0
        msg_vel.linear.z = 0
        msg_vel.angular.x = 0
        msg_vel.angular.y = 0
        msg_vel.angular.z = 0
        for i in range(20):
            self.pub_cmd_vel.publish(msg_vel)
            rospy.sleep(0.025)

    def callback_amcl_pose(self, msg):
        """gets covariance of amcl pose to check if already localized

        Args:
            msg (_type_): geometry_msgs/PoseWithCovarianceStamped
        """
        self.cov = np.amax(msg.pose.covariance)
        msg_bool = Bool()
        if self.cov < self.cov_max :
            msg_bool.data = True
        else:
            msg_bool.data = False

        self.pub_amcl_status.publish(msg_bool)


    def delete_costmap(self):
        """delete costmap of move_base

        Returns:
            _type_: _description_
        """
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            empty = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            resp = empty()
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
if __name__ == '__main__':
    rospy.init_node('navigation_goal_stop_node')
    wcr_nav = NavHandler()
    rospy.spin()
