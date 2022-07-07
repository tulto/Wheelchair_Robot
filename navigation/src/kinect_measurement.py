#!/usr/bin/env python3
from __future__ import print_function

from attr import dataclass

import roslib
import sys
import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class KinectMeasure:

  def __init__(self):
    self.image_pub = rospy.Publisher("/depth/std",Image, queue_size=2)
    #self.image_pub_2 = rospy.Publisher("/ir/image_raw2",Image, queue_size=5)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback_depth_image)
    rospy.Timer(rospy.Duration(1), self.callback_median)
    rospy.Timer(rospy.Duration(1), self.callback_std)
    self.images = [[]]
    self.data_size = 8
    self.first = True
    self.image_name_0 = "depth_img_0.jpg"
    self.image_name_1 = "depth_img_1.jpg"
    self.image_depth_diff_combined_name = "depth_diff_combined.jpg"


  def save_depth_image_msg(self, msg, name = "depth_img.jpg"):

    try:
      self.first = False
      depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")  
    except CvBridgeError as e:
      print(e)

    depth_array = np.array(depth_image, dtype=np.float32)
    #np.save("depth_img.npy", depth_array)
    #rospy.loginfo(depth_array)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.06), cv2.COLORMAP_JET)
    cv2.imwrite(name, depth_colormap)
    

  def colour_legend(self, alpha_=0.06):
    # Make an array of 120000 random bytes
    randomByteArray = bytearray(os.urandom(25500))
    # translate into numpy array
    flatNumpyArray = np.array(randomByteArray)
    #Convert the array to make a 400 * 300 grayscale image
    grayImage = flatNumpyArray.reshape(100, 255)
    for i in range (0, 255):
      for x in range(0, 100):
        grayImage[x][i] = i*255
    # show gray image
    colourImage = cv2.applyColorMap(cv2.convertScaleAbs(grayImage, alpha=255/grayImage.max()), cv2.COLORMAP_JET)
    cv2.imshow('GrayImage', colourImage)


  def callback_depth_image(self, msg):
    ksize = (3, 3)
    if self.first:

      # first_images will be initialized if not availabel
      self.save_depth_image_msg(msg, "depth_img_0.jpg")   
      #self.save_depth_image_msg(msg, "depth_diff_combined.jpg")
      #self.save_depth_image_msg(msg, "depth_diff_enhanced_combined.jpg")
      image_for_array = cv2.imread("depth_img_0.jpg")

      # initialize array of images
      depth_array = np.array(image_for_array, dtype=np.float32)
      self.images = [depth_array]

    else:
      self.first = False
      # read and blur images
      self.save_depth_image_msg(msg, "depth_img_1.jpg")
      image_1 = cv2.imread('depth_img_1.jpg')
      image_1_blur = cv2.blur(image_1, ksize)
      image_0 = cv2.imread('depth_img_0.jpg')
      image_0_blur = cv2.blur(image_0, ksize)

      # adding array of images
      depth_array_image_1 = np.array(image_1_blur, dtype=np.float32)
      self.images = np.append(self.images, [depth_array_image_1], axis = 0)
      if len(self.images)>self.data_size: # max array size
        self.images = np.delete(self.images, 0,0)
      
      # calculate std_array
      std_per_pixel = np.std(self.images, 0)
      std = np.std(self.images)

      #std_image
      std_multiplicator = 127.5
      std_image_gray = cv2.cvtColor(std_per_pixel*std_multiplicator, cv2.COLOR_BGR2GRAY) # x51~5mm x127.5~2mm x170~1.5mm 
      #std_image_gray_enhanced = cv2.equalizeHist(cv2.convertScaleAbs(std_image_gray))
      std_image_color = cv2.applyColorMap(cv2.convertScaleAbs(std_image_gray, alpha=1), cv2.COLORMAP_JET)
      #std_image_name = "std_picture_std_max_"
      #std_image_name += str(255/std_multiplicator)
      cv2.imwrite("offset_image.jpg", std_image_color)

      # calculate median_array
      median_array = np.median(self.images, 0)
      median = np.median(median_array)

      # median image
      median_multiplicator = 1
      subtractred_median_array= (median_array-215)*12.75
      #median_image_gray = cv2.cvtColor((median_array), cv2.COLOR_BGR2GRAY)
      #median_image_color = cv2.applyColorMap(cv2.convertScaleAbs(median_image_gray, alpha=1), cv2.COLORMAP_JET)
      median_subtracted_gray = cv2.cvtColor(subtractred_median_array, cv2.COLOR_BGR2GRAY)
      median_subtracted_color = cv2.applyColorMap(cv2.convertScaleAbs(median_subtracted_gray, alpha=1), cv2.COLORMAP_JET)
      cv2.imwrite("std_picture__max_2mm.jpg", median_subtracted_color)

      # calculating image difference
      image_diff = cv2.subtract(image_0_blur, image_1_blur)
      self.save_depth_image_msg(msg, "depth_img_0.jpg")

      #convert color image to gray
      image_diff_gray = cv2.cvtColor(image_diff, cv2.COLOR_BGR2GRAY)

      # calculate data 
      depth_array = np.array(image_diff, dtype=np.float32)
      dist_min = np.amin(depth_array)
      dist_max = np.amax(depth_array)
      std_max = np.amax(std_per_pixel)
      std_min = np.amin(std_per_pixel)
      #std_median = np.median(std_per_pixel)

      # print out info to picture
      print("dist_max:", dist_max)
      print("dist_min:", dist_min)
      print("median_dist", median)
      print("whole_std: ", std)
      print("std_max: ", std_max)
      print("std_min", std_min)
      #print(self.images)


      #adding higher contrast to gray image
      #image_diff_gray_enhanced = cv2.equalizeHist(image_diff_gray)
      #image_diff_color_enhanced = cv2.applyColorMap(cv2.convertScaleAbs(image_diff_gray_enhanced, alpha=0.75), cv2.COLORMAP_JET)

      
      #add(or blend) the images
      #image_diff_color_saved = cv2.imread("depth_diff_enhanced_combined.jpg")
      #image_diff_color_merged = cv2.addWeighted(image_diff_color_enhanced, 0.2, image_diff_color_saved, 0.8, 0)
      #cv2.imwrite("depth_diff_enhanced_combined.jpg", image_diff_color_merged)
      

      #show diffrent images
      cv2.imshow("Original", image_0)
      #cv2.imshow("Image_Difference", image_diff)
      #cv2.imshow("Image_Difference_enhanced", image_diff_color_merged)
      cv2.imshow("std_per_pixel", std_image_color)
      cv2.imshow("image-median", median_subtracted_color)
      self.colour_legend(0.06)
      cv2.waitKey(3) 

  def callback_median(self, event):
    pass

  def callback_std(self, event):
    pass
    




def main(args):
  ic = KinectMeasure()
  rospy.init_node('kinect_measurement_node')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('kinect_measurement_node')
    kinect_measure = KinectMeasure()
    rospy.spin()