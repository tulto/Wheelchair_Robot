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

import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import imutils


class KinectMeasure:

  def __init__(self):
    self.image_pub = rospy.Publisher("/depth/std",Image, queue_size=2)
    #self.image_pub_2 = rospy.Publisher("/ir/image_raw2",Image, queue_size=5)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback_depth_image)
    #self.image_depth_sub = rospy.Subscriber("/camera/depth_registered/image",Image,self.callback_image_2_depth)
    #self.depth_img_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.callback_cloud_image)
    rospy.Timer(rospy.Duration(2), self.callback_median)
    rospy.Timer(rospy.Duration(2), self.callback_std)
    self.first = True       # initializing if no images exist prior to calculation
    self.images = [[]]      # saving 4D-Array of images
    self.data_size = 10     # max image data size
    self.median_multiplicator = 5   # use of multiplicator to intesifie colormap median_image: (factor)/(max_color_value) 5/40cm  10/20cm   25/8cm
    self.std_multiplicator = 100    # use of multiplicator to intesifie colormap sdt_image: (factor)/(max_color_value) 20/10cm  50/4cm  100/2cm  200/1cm
    self.image_name_0 = "depth_img_0.jpg"
    self.image_name_1 = "depth_img_1.jpg"
    self.image_depth_diff_combined_name = "depth_diff_combined.jpg"


  def save_depth_image_msg(self, msg, name = "depth_img.jpg"):
    """A funktion to save the ROS_image_msg to a .jpg file + converting it to colormap

    Args:
        msg (_type_): _description_
        name (str, optional): _description_. Defaults to "depth_img.jpg".
    """
    try:
      self.first = False
      depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")  
    except CvBridgeError as e:
      print(e)

    depth_array = np.array(depth_image, dtype=np.float32)
    #np.save("depth_img.npy", depth_array)
    #rospy.loginfo(depth_array)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_array, alpha=0.06), cv2.COLORMAP_JET)
    cv2.imwrite(name, depth_colormap)
    

  def colour_legend(self):
    """generates and saves a collour legend of the used colormap
    """
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
    cv2.imwrite("colour_legend.jpg", colourImage)


  def callback_depth_image(self, msg):
    """generating, out of ROS_msgs, an 4D-image-array to save multiple images. Also shows original images in opencv2 

    Args:
        msg (_type_): _description_
    """
    ksize = (3, 3)
    if self.first:

      # first_images will be initialized if not availabel
      self.save_depth_image_msg(msg, "depth_img_0.jpg")   
      #self.save_depth_image_msg(msg, "depth_diff_combined.jpg")
      #self.save_depth_image_msg(msg, "depth_diff_enhanced_combined.jpg")
      image_for_array = cv2.imread("depth_img_0.jpg")

      # initialize array of images
      depth_array = np.array(image_for_array, dtype=np.float32)
      #np.save("depth_img_1.npy", depth_array)
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
      depth_array_image_1 = np.array(image_1, dtype=np.float32)
      self.images = np.append(self.images, [depth_array_image_1], axis = 0)
      if len(self.images)>self.data_size: # max array size
        self.images = np.delete(self.images, 0,0)
      

      # calculating image difference
      image_diff = cv2.subtract(image_0_blur, image_1_blur)
      self.save_depth_image_msg(msg, "depth_img_0.jpg")
  
      #show diffrent images
      cv2.imshow("Original", image_0)
      
      self.colour_legend()
      cv2.waitKey(3) 



  def callback_median(self, event):
    """generates a median image over multiple saved images of an 4D-array. 
    It is then subtracted with the point in the middle to get a distotion of distances 

    Args:
        event (_type_): _description_
    """
    # calculate median_array
    median_array = np.median(self.images, 0)
    median = np.median(median_array)
    #test = cv2.cvtColor(median_array, cv2.COLOR_BGR2GRAY)
    #np.save("depth_img_1.npy", median_array)
    mid_x = len(median_array)/2
    mid_y = len(median_array[0])/2
    middle_point = median_array[240][320]
    print("middle_point: ", middle_point)
    #cv2.imwrite("test.jpg", test)

    # median image
    subtractred_median_array= (median_array-middle_point)*self.median_multiplicator
    median_subtracted_gray = cv2.cvtColor(subtractred_median_array, cv2.COLOR_BGR2GRAY)
    median_subtracted_color = cv2.applyColorMap(cv2.convertScaleAbs(median_subtracted_gray, alpha=1), cv2.COLORMAP_JET)
    offset_image_name = "offset_image_max_"
    offset_image_name += str(200/self.median_multiplicator)
    offset_image_name += "cm.jpg"

    cv2.imwrite(offset_image_name, median_subtracted_color)

    print("median_dist", median)

    cv2.imshow("image-median", median_subtracted_color)
    cv2.waitKey(3) 




  def callback_std(self, event):
    """Calculates the std_image over multiple images in an 4D-array. 
    The image is then shown and will be saved over opencv2

    Args:
        event (_type_): _description_
    """
    #0.00375
    std_80_test_0 = [0.8225811719894409, 0.8167948126792908, 0.8161888718605042, 0.8167234659194946, 0.8161196708679199, 0.8155173659324646, 0.8149166107177734, 0.8137195706367493, 0.8131232857704163, 0.8125286102294922, 0.8119354248046875, 0.811343789100647, 0.8096228837966919, 0.809578001499176, 0.8089925050735474, 0.8084085583686829, 0.8078261613845825]
    
    #0.001309
    std_80_test_1 = [0.7127206325531006, 0.7126125693321228, 0.7104995846748352, 0.7124029994010925, 0.71230149269104, 0.7122021913528442, 0.7100990414619446, 0.7120100855827332, 0.7099118232727051, 0.7098215222358704, 0.7097333073616028, 0.7096473574638367, 0.7095635533332825, 0.7094818949699402, 0.7094024419784546, 0.7093251943588257, 0.7092501521110535, 0.7081755995750427, 0.7091065049171448, 0.7090380191802979, 0.7089716792106628, 0.7089074850082397, 0.7088455557823181, 0.7087857723236084, 0.7087281346321106, 0.7086727619171143, 0.7086195349693298, 0.7085685133934021, 0.708519697189331, 0.7084730267524719, 0.7084285616874695, 0.7083863019943237, 0.7083462476730347, 0.7083083987236023, 0.7082726955413818, 0.7072388529777527, 0.708207905292511, 0.7081787586212158, 0.7071516513824463, 0.7081271409988403, 0.7081046104431152, 0.7080842852592468, 0.7080661654472351, 0.7080501914024353, 0.708036482334137, 0.7080249190330505, 0.7080155611038208, 0.7080084085464478, 0.7100034356117249, 0.7100006937980652, 0.7100000977516174, 0.7080016732215881, 0.7080054879188538, 0.7080115079879761, 0.7080197334289551, 0.708030104637146, 0.7100428342819214, 0.7100576758384705, 0.7100747227668762, 0.7080936431884766, 0.710115373134613, 0.7081385850906372, 0.7081643342971802, 0.710192859172821, 0.7102230787277222]
    
    #0.011823
    std_190_test_0 = [2.1638453006744385, 2.1252636909484863, 2.1464855670928955, 2.1221141815185547, 2.143310308456421, 2.130354404449463, 2.1287848949432373, 2.1256580352783203, 2.1354410648345947, 2.1338791847229004, 2.132321357727051, 2.1194517612457275]

    #0.006166
    std_190_test_1 = [1.916723370552063, 1.9163405895233154, 1.9270328283309937, 1.9155924320220947, 1.9152270555496216, 1.9259302616119385, 1.9255744218826294, 1.9252245426177979, 1.9248803853988647, 1.9245420694351196, 1.9131566286087036, 1.9128319025039673, 1.912513017654419, 1.9121999740600586, 1.9118927717208862, 1.9115912914276123, 1.911295771598816, 1.911005973815918, 1.9107221364974976, 1.9104440212249756, 1.9101718664169312, 1.9099054336547852, 1.9096449613571167, 1.9093903303146362, 1.9091415405273438, 1.9088984727859497, 1.91968834400177, 1.9084302186965942, 1.9192291498184204, 1.9079853296279907, 1.9077715873718262, 1.9075639247894287, 1.9073619842529297, 1.9181842803955078, 1.9069757461547852, 1.917807698249817, 1.9066131114959717, 1.9064404964447021, 1.9062738418579102, 1.9061131477355957, 1.916969656944275, 1.9058092832565308, 1.9166758060455322, 1.9165377616882324, 1.9164056777954102, 1.905272126197815, 1.9161592721939087, 1.9050389528274536, 1.9049311876296997, 1.9048293828964233, 1.9157376289367676, 1.915647029876709, 1.9045591354370117]
    
    #0.01280
    std_190_test_2 = [2.1992621421813965, 2.1720354557037354, 2.170423984527588, 2.168816566467285, 2.167213201522827, 2.165613889694214, 2.164018392562866, 2.160839796066284, 2.147915840148926]

    #print("std_test: ", np.std(std_190_test_2))

    # calculate std_array
    std_per_pixel = np.std(self.images, 0)
    std = np.std(self.images)


    #std_image
    std_image_gray = cv2.cvtColor(std_per_pixel*self.std_multiplicator, cv2.COLOR_BGR2GRAY) 
    #std_image_gray_enhanced = cv2.equalizeHist(cv2.convertScaleAbs(std_image_gray))
    std_image_color = cv2.applyColorMap(cv2.convertScaleAbs(std_image_gray, alpha=1), cv2.COLORMAP_JET)
    std_image_name = "std_picture_std_max_"
    std_image_name += str(200/self.std_multiplicator)
    std_image_name += "cm.jpg"
    cv2.imwrite(std_image_name, std_image_color)
    
    std_max = np.amax(std_per_pixel)
    std_min = np.amin(std_per_pixel)

    print("whole_std: ", std)
    print("std_max: ", std_max)
    print("std_min", std_min)
    print("std_middle_point", std_per_pixel[240][320])

    cv2.imshow("std_per_pixel", std_image_color)
    cv2.waitKey(3) 
    




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