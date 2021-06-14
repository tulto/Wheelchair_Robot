#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
from services_and_messages.msg import Echosensors       #importing selfmade message type
from services_and_messages.srv import WarningEchoTest   #importing selfmade service type
from echo_sensor_class import EchoSensor                #import EchoSensor class
from request_warning import WarningRequest              #import WarningRequest class


def handle_warning_test(req): 
    if req.sensor == "el":
        WarningRequest.set_received_warning_request([False, (not WarningRequest.received_warning_request()[1]), False, False])
        WarningRequest.set_requested_warning([False, True, False, False])

        if not WarningRequest.received_warning_request()[1]:
            WarningRequest.setback_requested_warning([True, False, True, True])

        return True
    
    elif req.sensor == "er":
        WarningRequest.set_received_warning_request([False, False, (not WarningRequest.received_warning_request()[2]), False])
        WarningRequest.set_requested_warning([False, False, True, False])

        if not WarningRequest.received_warning_request()[2]:
            WarningRequest.setback_requested_warning([True, True, False, True])

        return True
    
    elif req.sensor == "ef":
        WarningRequest.set_received_warning_request([(not WarningRequest.received_warning_request()[0]), False, False, False])
        WarningRequest.set_requested_warning([True, False, False, False])

        if not WarningRequest.received_warning_request()[0]:
            WarningRequest.setback_requested_warning([False, True, True, True])

        return True
    
    elif req.sensor == "eb":
        WarningRequest.set_received_warning_request([False, False, False, (not WarningRequest.received_warning_request()[3])])
        WarningRequest.set_requested_warning([False, False, False, True])

        if not WarningRequest.received_warning_request()[3]:
            WarningRequest.setback_requested_warning([True, True, True, False])
            
        return True
    
    else:
        WarningRequest.setback_all_values()
        rospy.loginfo("False Input. Please send: \nel for Echo-Left\ner for Echo-Right\nef for Echo-Front\neb for Echo-Back")
        return False


def echo_warning_direction(echo_warning):
    warning_list = [True, True, True, True]
    
    #decide warning for leftside echo sensors
    if echo_warning[0] or echo_warning[1]:
        warning_list[1] = True
    else:
        warning_list[1] = False
    
    #decide warning for rightside echo sensors
    if echo_warning[2] or echo_warning[3]:
        warning_list[2] = True
    else:
        warning_list[2] = False
    
    #front echo warning
    warning_list[0] = echo_warning[4]
    
    #back echo warning
    warning_list[3] = echo_warning[5]
    
    return warning_list

#########################################################################
            #Main-Code following
#########################################################################

if __name__ == '__main__':
    rospy.init_node("echo_sensor_publisher")
    
    #the following names are short for:
    #e = echo, l = left, r = right, f = front, b = back
    #so for examle elf meand echo left front 
    #the front echosensor on the left side of the robot
    #(front is seen from seating position)

    EchoSensor.setmode()
    
    #initialising trigger_pin
    EchoSensor.init_trigger_pin()
    
    sensor_list = [
    EchoSensor(21),    #elf  #Pins müssen noch überprüft werden
    EchoSensor(6),     #elb
    EchoSensor(25),    #erf
    EchoSensor(26),    #erb
    EchoSensor(24),    #ef
    EchoSensor(5)      #eb
    ]

    #initialising all sensor pins:
    for sensor in sensor_list:
        sensor.init_sensor()

    sensor_dict = {
            sensor_list[0] : "Echo-Left-Front",
            sensor_list[1] : "Echo-Left-Back",
            sensor_list[2] : "Echo-Right-Front",
            sensor_list[3] : "Echo-Right-Back",
            sensor_list[4] : "Echo-Front",
            sensor_list[5] : "Echo-Back",
        }
    
    #implementing the publisher for the echo sensors warning
    pub = rospy.Publisher("echo_sensor", Echosensors, queue_size=10)
    publish_frequency = 2
    rate = rospy.Rate(publish_frequency)
    
    #service server initializing
    test_warning_service = rospy.Service("/echo_test_warning", WarningEchoTest, handle_warning_test)

    while not rospy.is_shutdown():
        echo_msg = Echosensors()    #creating an instance of the Echosensor() message type

        for sensor in sensor_list:
            rospy.loginfo("Measured " + str(sensor.get_distance_in_m()) + " m from " + str(sensor_dict[sensor]))
        
        #looking if one echo sensor senses a distance -
        #that would result in a warning and publishing
        echo_warning_list = [True, True, True, True, True, True] #creating a boolean list that saves the received warnings -
        
        #for safety purposes initialized with all values on true
        for sensor in range(len(sensor_list)):
            echo_warning_list[sensor] = sensor_list[sensor].distance_warning(0.30, 0.35)
    
        echo_msg.echo_dir = echo_warning_direction(echo_warning_list)

        #overwriting the message file if WarningRequest has been received
        if any(WarningRequest.received_warning_request()):
            echo_msg.echo_dir = WarningRequest.requested_warning_msg()

        pub.publish(echo_msg)
        rate.sleep()

    if rospy.is_shutdown():
        for sensor in sensor_list:
            sensor.reset_echo_pins()
    