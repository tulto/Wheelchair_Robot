#!/usr/bin/env python3

from xxlimited import foo
import numpy as np
PACKAGE = '/move_base/local_costmap'
#import roslib;roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.client
from nav_msgs.msg import Odometry
from services_and_messages.srv import MovementCheck
from dynamic_reconfigure.server import Server
from navigation.cfg import DynamicFootprintConfig


class DynamicFoot:
    def __init__(self):
        self.sub_stop = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)
        self.srv = Server(DynamicFootprintConfig, self.dynamic_param)
        self.client = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=30)
        rospy.Timer(rospy.Duration(2), self.callback_dynamic_set)

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
            foot = rospy.get_param("move_base/global_costmap/footprint")
            padding_below = rospy.get_param("dynamic_footprint_node/scaling_below")
            padding_above = rospy.get_param("dynamic_footprint_node/scaling_over")
            drive_right = rospy.get_param("dynamic_footprint_node/drive_right")
            #if (vel < scaling_vel):
            #    padding = rospy.get_param("dynamic_footprint_node/scaling_below")
            #else:
            #    padding = rospy.get_param("dynamic_footprint_node/scaling_over")
            #
            #self.client.update_configuration({"footprint_padding":padding})

            client_movement_check = rospy.ServiceProxy('check_movement', MovementCheck)
            x1 = client_movement_check(0.2,0,0)
            x2 = client_movement_check(-0.2,0,0)
            y1 = client_movement_check(0,0.05,0)
            y2 = client_movement_check(0,0.1,0)
            y3 = client_movement_check(0,0.15,0)

            if drive_right & x1.check & x2.check & y3.check & (vel > scaling_vel):
                self.client.update_configuration({"footprint_padding":padding_below})
                self.client.update_configuration({"footprint":[[-0.75,-0.37],[-0.75,0.67],[0.7,0.67],[0.7,-0.37]]})

            elif drive_right & x1.check & x2.check & y2.check & (vel > scaling_vel):
                self.client.update_configuration({"footprint_padding":padding_below})
                self.client.update_configuration({"footprint":[[-0.75,-0.37],[-0.75,0.57],[0.7,0.57],[0.7,-0.37]]})

            elif drive_right & x1.check & x2.check & y1.check & (vel > scaling_vel):
                self.client.update_configuration({"footprint_padding":padding_below})
                self.client.update_configuration({"footprint":[[-0.75,-0.37],[-0.75,0.47],[0.7,0.47],[0.7,-0.37]]})

            elif vel < scaling_vel:
                self.client.update_configuration({"footprint":foot})
                self.client.update_configuration({"footprint_padding":padding_below})
            else:
                self.client.update_configuration({"footprint":foot})
                self.client.update_configuration({"footprint_padding":padding_above})

            

                
            
    def dynamic_param(self, config, level):
        rospy.loginfo("""Reconfigure Request: {dynamic_footprint}, {scaling_below},\ 
            {scaling_over}, {scaling_vel}""".format(**config))

        return config


        
if __name__ == '__main__':
    rospy.init_node('dynamic_footprint_node')
    wcr_dyn_pad = DynamicFoot()
    rospy.spin()