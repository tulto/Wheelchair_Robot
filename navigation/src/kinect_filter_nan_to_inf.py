#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan 

class kinect_scan_nan_to_inf:

  def __init__(self):
    self.scan_pub = rospy.Publisher("/kinect_scan_nan_to_inf",LaserScan, queue_size=2)
    self.scan_sub = rospy.Subscriber("/kinect_scan",LaserScan,self.callback)

  def callback(self,msg):

    msg_kinect = LaserScan()
    msg_kinect.header = msg.header
    msg_kinect.angle_max = msg.angle_max
    msg_kinect.angle_min = msg.angle_min
    msg_kinect.angle_increment = msg.angle_increment
    msg_kinect.time_increment = msg.time_increment
    msg_kinect.scan_time = msg.scan_time
    msg_kinect.range_min = msg.range_min
    msg_kinect.range_max = msg.range_max


    for i in range(0,len(msg.ranges)-1):
      if (math.isnan(msg.ranges[i])):
        print(len(msg.ranges))
        msg_kinect.ranges.append(math.inf)
      else:
        msg_kinect.ranges.append(msg.ranges[i])
        

    self.scan_pub.publish(msg_kinect)

    

if __name__ == '__main__':
    rospy.init_node('kinect_scan_nan_to_inf')
    kinect_filter = kinect_scan_nan_to_inf()
    rospy.spin()