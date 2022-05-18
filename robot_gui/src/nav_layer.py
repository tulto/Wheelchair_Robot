#!/usr/bin/env python3

import numpy as np
import rospy
import time
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_srvs.srv import Empty


class NavHandler:
    def __init__(self):
        self.sub_stop = rospy.Subscriber("/nav_cancle", String, self.callback_cancel)
        self.pub_stop = rospy.Publisher("/move_base/cancel", GoalID, queue_size=5)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.sub_goal = rospy.Subscriber("/nav_goal", String, self.callback_goal) 
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)

        #self.sub_locate = rospy.Subscriber("/nav_locate", String, self.callback_locate) 
        #self.pub_locate = rospy.Publisher("/nav_locate_status", String, queue_size=5)

        self.sub_amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_amcl_pose)
        self.pub_amcl_status = rospy.Publisher("/amcl_status", Bool, queue_size=5)
        self.cov = 100
        self.cov_max = 0.1

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
        if self.cov < self.cov_max:
            self.pub_goal.publish(msg_goal)

    """"    
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
        """simple cancle option of navigation

        Args:
            msg (_type_): std_msgs/String
        """
        msg_stop = GoalID()
        msg_stop.stamp.nsecs = 0
        msg_stop.stamp.secs = 0
        msg_stop.id = ""
        self.pub_stop.publish(msg_stop)

        # publish cmd_vel 0
        time.sleep(100)
        msg_vel = Twist()
        msg_vel.linear.x = 0
        msg_vel.linear.y = 0
        msg_vel.linear.z = 0
        msg_vel.angular.x = 0
        msg_vel.angular.y = 0
        msg_vel.angular.z = 0
        self.pub_cmd_vel.publish(msg_vel)

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
