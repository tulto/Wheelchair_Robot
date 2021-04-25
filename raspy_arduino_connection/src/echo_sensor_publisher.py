#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
#importing selfmase message type
from services_and_messages.msg import Echosensors 
from services_and_messages.srv import WarningEchoTest
from echo_sensor_class import EchoSensor


def handle_warning_test(req):
    if req.sensor == "elf":
        echo_msg.echolf=True
        return True
    elif req.sensor == "elb":
        echo_msg.echolb=True
        return True
    elif req.sensor == "erf":
        echo_msg.echorf=True
        return True
    elif req.sensor == "erb":
        echo_msg.echorb=True
        return True
    elif req.sensor == "ef":
        echo_msg.echof=True
        return True
    elif req.sensor == "eb":
        echo_msg.echob=True
        return True
    else:
        rospy.loginfo("False Input. Please send: \nelf for Echo-Left-Front\nelb for Echo-Left-Back\nerf for Echo-Right-Front\nerb for Echo-Right-Back\nef for Echo-Front\neb for Echo-Back")
        return False




if __name__ == '__main__':
    rospy.init_node("echo_sensor_publisher")
    
    pub = rospy.Publisher("echo_sensor", echosensor, queue_size=10)
    publish_frequency = 4
    rate = rospy.Rate(publish_frequency)
    #the following names are short for:
    # e = echo, l = left, r = right, f = front, b = back
    #so for examle elf meand echo left front 
    #the front echosensor on the left side of the robot
    #(front is seen from seating position)
    sensor_list = [
    EchoSensor(2,3),  #elf  #Pins müssen noch überprüft werden
    EchoSensor(17,27),#elb
    EchoSensor(5,6),  #erf
    EchoSensor(16,12),#erb
    EchoSensor(8,7),  #ef
    EchoSensor(23,24) #eb
    ]
    #initialising all sensor pins:
    for sensor in sensor_list:
        sensor.init_sensor()
    
    ''' #maybe needed later after test
    elf.init_sensor()
    elb.init_sensor()
    erf.init_sensor()
    erb.init_sensor()
    ef.init_sensor()
    eb.init_sensor()
    '''
    sensor_dict = {
            sensor_list[0] : "Echo-Left-Front",
            sensor_list[1] : "Echo-Left-Back",
            sensor_list[2] : "Echo-Right-Front",
            sensor_list[3] : "Echo-Right-Back",
            sensor_list[4] : "Echo-Front",
            sensor_list[5] : "Echo-Back",
        }

    ''' #while loop dosent work for a pulisher and a server
    while not rospy.is_shutdown(): #muss mit spin umgesetzt werden!!!
        echo_msg = Echosensors()

        for sensor in sensor_list:
            rospy.loginfo("Measured " + str(sensor.get_distance_in_m()) + " from " + sensor_dict(sensor))
        
        #looking if one echo sensor senses a distance
        #that would result in a warning and publishing it
        #to the arduino mega2560
        echo_msg.echolf = sensor_list[0].distance_warning()
        echo_msg.echolb = sensor_list[1].distance_warning()
        echo_msg.echorf = sensor_list[2].distance_warning()
        echo_msg.echorb = sensor_list[3].distance_warning()
        echo_msg.echof = sensor_list[4].distance_warning()
        echo_msg.echob = sensor_list[5].distance_warning()
        
        #test functionality service code under here
        test_warning_service = rospy.Service("/echo_test_warning", WarningEchoTest, handle_warning_test)

        pub.publish(echo_msg)
        rate.sleep()
    '''
    echo_msg = Echosensors()

    for sensor in sensor_list:
        rospy.loginfo("Measured " + str(sensor.get_distance_in_m()) + " from " + sensor_dict(sensor))
        
    #looking if one echo sensor senses a distance
    #that would result in a warning and publishing it
    #to the arduino mega2560
    echo_msg.echolf = sensor_list[0].distance_warning()
    echo_msg.echolb = sensor_list[1].distance_warning()
    echo_msg.echorf = sensor_list[2].distance_warning()
    echo_msg.echorb = sensor_list[3].distance_warning()
    echo_msg.echof = sensor_list[4].distance_warning()
    echo_msg.echob = sensor_list[5].distance_warning()
        
    #test functionality service code under here
    test_warning_service = rospy.Service("/echo_test_warning", WarningEchoTest, handle_warning_test)

    pub.publish(echo_msg)
    rate.sleep()
    rospy.spin()

    for sensor in sensor_list:
        sensor.reset_echo_pins()

    ''' #maybe needed later after test
    elf.reset_echo_pins()
    elb.reset_echo_pins()
    erf.reset_echo_pins()
    erb.reset_echo_pins()
    ef.reset_echo_pins()
    eb.reset_echo_pins()
    '''