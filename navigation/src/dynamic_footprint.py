#!/usr/bin/env python3

import numpy as np
PACKAGE = '/move_base/local_costmap'
#import roslib;roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.client
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from navigation.cfg import DynamicFootprintConfig


class DynamicFoot:
    def __init__(self):
        self.sub_stop = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.srv = Server(DynamicFootprintConfig, self.dynamic_param)
        self.client = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=30)
        rospy.Timer(rospy.Duration(3), self.callback_dynamic_set)

        self.vel_msg = Odometry()

    def callback_odom(self, msg):
        """get odom_msg values

        Args:
            msg (_type_): _description_
        """
        self.vel_msg = msg 
        

    def callback_dynamic_set(self, event):
        """set the footprint padding factor to dynamic to velocity

        Args:
            event (_type_): _description_
        """
        
        if (rospy.get_param("dynamic_footprint_node/dynamic_footprint")):
            vel = np.sqrt(self.vel_msg.twist.twist.linear.x**2 + self.vel_msg.twist.twist.linear.y**2) 
            print(vel)
            scaling_vel = rospy.get_param("dynamic_footprint_node/scaling_vel")
            if (vel < scaling_vel):
                padding = rospy.get_param("dynamic_footprint_node/scaling_below")
            else:
                padding = rospy.get_param("dynamic_footprint_node/scaling_over")
            
            self.client.update_configuration({"footprint_padding":padding})

                
            
    def dynamic_param(self, config, level):
        rospy.loginfo("""Reconfigure Request: {dynamic_footprint}, {scaling_below},\ 
            {scaling_over}, {scaling_vel}""".format(**config))

        return config


        
if __name__ == '__main__':
    rospy.init_node('dynamic_footprint_node')
    wcr_dyn_pad = DynamicFoot()
    rospy.spin()