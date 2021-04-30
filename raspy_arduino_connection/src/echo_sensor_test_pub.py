#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
from services_and_messages.msg import Echosensors #importing selfmade message type
from services_and_messages.srv import WarningEchoTest #importing selfmade service type
from echo_sensor_class import EchoSensor #import EchoSensor class


def handle_warning_test(req):
    if req.sensor == "el":
        echo_msg.echo_dir[1]=True
        return True
    elif req.sensor == "er":
        echo_msg.echo_dir[2]=True
        return True
    elif req.sensor == "ef":
        echo_msg.echo_dir[0]=True
        return True
    elif req.sensor == "eb":
        echo_msg.echo_dir[3]=True
        return True
    else:
        rospy.loginfo("False Input. Please send: \nel for Echo-Left\ner for Echo-Right\nef for Echo-Front\neb for Echo-Back")
        return False

def echo_warning_direction(echo_warning):
    warning_list[4]
    #decide warning for leftside echo sensors
    if echo_warning[0]:
        warning_list[1] = True
    else:
        warning_list[1] = False
    #decide warning for rightside echo sensors
    if echo_warning[1]:
        warning_list[2] = True
    else:
        warning_list[2] = False
    #front echo warning
    warning_list[0] = echo_warning[2]
    #back echo warning
    warning_list[3] = echo_warning[3]
    
    return warning_list


if __name__ == '__main__':
    rospy.init_node("echo_sensor_publisher")
    
    #the following names are short for:
    # e = echo, l = left, r = right, f = front, b = back
    #so for examle elf meand echo left front 
    #the front echosensor on the left side of the robot
    #(front is seen from seating position)
    EchoSensor.setmode()
    sensor_list = [
    #EchoSensor(2,3),  #elf  #Pins müssen noch überprüft werden
    #EchoSensor(17,27),#elb
    EchoSensor(5,6),  #erf
    EchoSensor(16,12),#erb
    EchoSensor(8,7),  #ef
    EchoSensor(23,24) #eb
    ]
    #initialising all sensor pins:
    for sensor in sensor_list:
        sensor.init_sensor()

    sensor_dict = {
            sensor_list[0] : "Echo-Left",
            sensor_list[1] : "Echo-Right",
            sensor_list[2] : "Echo-Front",
            sensor_list[3] : "Echo-Back",
        }
    
    #implementing the publisher for the echo sensors warning
    pub = rospy.Publisher("echo_sensor", Echosensors, queue_size=10)
    publish_frequency = 2
    rate = rospy.Rate(publish_frequency)

    echo_msg = Echosensors() #creating an instance of the Echosensor() message type

    for sensor in sensor_list:
        rospy.loginfo("Measured " + str(sensor.get_distance_in_m()) + " from " + str(sensor_dict[sensor]))
        
    #looking if one echo sensor senses a distance
    #that would result in a warning and publishing
    echo_warning_list = [True, True, True, True] #creating a boolean list that saves the received warnings
    #for safety purposes initialized with all values on true
    for sensor in range(sensor_list):
        echo_warning_list[sensor] = sensor_list[sensor].distance_warning(0.2, 0.25)
    
    echo_msg.echo_dir = echo_warning_direction(echo_warning_list)

    #service server initializing, needs to be after the echo_msg.echo_dir=echo_warning_direction(echo_warning_list) command to overwrite the msg values for the test
    test_warning_service = rospy.Service("/echo_test_warning", WarningEchoTest, handle_warning_test)

    pub.publish(echo_msg)
    rate.sleep()
    rospy.spin()

    if rospy.is_shutdown():
        for sensor in sensor_list:
            sensor.reset_echo_pins()
    