#!/usr/bin/env python3
from __future__ import print_function

from attr import dataclass

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/ir/image_raw",Image, queue_size=2)
    #self.image_pub_2 = rospy.Publisher("/ir/image_raw2",Image, queue_size=5)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/ir/image_raw",Image,self.callback)

  def callback(self,data):

    msg = Image()
    msg = data
    #msg.step = 2

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono16")
    except CvBridgeError as e:
      print(e)


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    image_msg = cv2.convertScaleAbs(cv_image)


    image_msg = self.bridge.cv2_to_imgmsg(image_msg, "passthrough")
    image_msg.header = data.header
    image_msg.encoding = "mono8"


    try:
      self.image_pub.publish(image_msg)
    except CvBridgeError as e:
      print(e)

    #msg_image = Image()
    #msg_image = data
    #msg_image.header = data.header
    #msg_image.height = data.height
    #msg_image.width = data.width
    #msg_image.encoding = "mono16"
    #msg_image.is_bigendian = data.is_bigendian
    #msg_image.step = 640


    #print (len(data.data))
    #x = 0
    #for i in range (0, len(data.data), 2):
    #  msg_image.data[x] = data.data[i]
    #  x = x+1

    #print (len(msg_image.data))

    #self.image_pub_2.publish(msg_image)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)