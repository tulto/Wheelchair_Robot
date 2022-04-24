#!/usr/bin/env python3

import numpy as np
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_srvs.srv import Empty


class NavHandler:
    def __init__(self):
        self.sub_stop = rospy.Subscriber("/nav_cancle", String, self.callback_cancel)
        self.pub_stop = rospy.Publisher("/move_base/cancel", GoalID, queue_size=5)

        self.sub_goal = rospy.Subscriber("/nav_goal", String, self.callback_goal) 
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)

        self.sub_locate = rospy.Subscriber("/nav_locate", String, self.callback_locate) 
        self.pub_locate = rospy.Publisher("/nav_locate_status", String, queue_size=5)

        self.sub_amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_amcl_pose)
        self.cov = 100

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
        msg_goal.header.stamp = rospy.Time()
        msg_goal.header.stamp = rospy.Time()
        msg_goal.header.frame_id = "map"
        
        msg_goal.pose.position.x = pose[0]
        msg_goal.pose.position.y = pose[1]
        msg_goal.pose.position.z = pose[2]

        msg_goal.pose.orientation.x = pose[3]
        msg_goal.pose.orientation.y = pose[4]
        msg_goal.pose.orientation.z = pose[5]
        msg_goal.pose.orientation.w = pose[6]
        if (self.cov < 0.1):
            self.pub_goal.publish(msg_goal)

    def callback_locate(self, msg):
        """send a string with matching param to make locate movement

        Args:
            msg (_type_): std_msgs/String
        """

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


    def callback_cancel(self, msg):
        """simple cancle option of navigation

        Args:
            msg (_type_): std_msgs/String
        """
        msg_stop = GoalID()
        msg_stop.stamp.nsecs = 0
        msg_stop.stamp.secs = 0
        msg_stop.id = ""
        self.pub_stop.publish(msg_stop)

    def callback_amcl_pose(self, msg):
        """gets covariance of amcl pose to check if already localized

        Args:
            msg (_type_): geometry_msgs/PoseWithCovarianceStamped
        """
        self.cov = np.amax(msg.pose.covariance)

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