import numpy
import math

class RobotPosition:

    def __init__(self, x_coordinate, y_coordinate):
        self.x=x_coordinate
        self.y=y_coordinate
    
    #l = left, r = right, f = front, b = back, w = wheel, dir = direction
    #if dir (direction) is positive the wheel drives to forwards, if it is false backwards
    def odometry_calculation(self, lfw, lbw, rfw, rbw, lf_dir, lb_dir, rf_dir, rb_dir):

        encoder_resolution=500 #500 encoder steps per wheel revolution (NEEDS TO BE TESTS THIS VALUE IS/MIGHT BE WRONG)
        mecanumwheel_size = 0.2 #one full rotation results in 0.2 meters driven (NEEDS TO BE TESTET THIS VALUE IS/MIGHT BE WRONG)

        if lf_dir and lb_dir and rf_dir and rb_dir:
            rospy.loginfo("Robot is moving forwards!")
            self.x=0
            self.y=
        
