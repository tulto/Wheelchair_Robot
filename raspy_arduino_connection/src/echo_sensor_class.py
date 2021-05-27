import RPi.GPIO as GPIO
import time
import numpy

class EchoSensor:

    TRIG = 18 #Trigger-Pin for all Sensors
    TRIG_already_init = False

    def __init__(self, ECHOpin):
        #self.TRIG=TRIGERpin as test
        self.ECHO=ECHOpin
        self.recorded_distances=[0.0,0.0,0.0]

    def init_sensor(self):
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.remove_event_detect(self.ECHO)
        time.sleep(0.1) 

    #changing program to avoid multiple while loops
    def get_distance_in_m(self):

        pulse_start = 0.0
        pulse_end = 0.0
        pulse_duration = 0.0
        last_ECHO_value = False

        GPIO.output(self.TRIG,False)
        time.sleep(0.001)
        GPIO.output(self.TRIG,True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG,False)
        
        while pulse_duration <= 0.008734:
            if GPIO.input(self.ECHO) and not last_ECHO_value:
                pulse_start = time.time()
            elif not GPIO.input(self.ECHO) and last_ECHO_value:
                pulse_end = time.time()
            last_ECHO_value = GPIO.input(self.ECHO)
            pulse_duration = (pulse_end - pulse_start)
        '''
        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            if (pulse_duration >= 0.008734): #0.008734 secondes equals about 1.5 meters
                break #echo sensors only used as a warning stop so everything over 5 meters is not of interest
        '''
        temperature = 20.0 #(air)temperature in °C, assuming normal room temperature
        v_air = (331.5 +(0.6*temperature)) #speed of sound in m/s
        distance = round(((v_air/2)*pulse_duration), 4) #divide v_air by 2 because wave travels two time the distance
        self.recorded_distances.append(distance)
        self.recorded_distances.pop(0)
        return distance

    def distance_warning(self, instant_stop_distance, average_stop_distance):
        if (sum(self.recorded_distances)) != 0.0 and self.recorded_distances[len(self.recorded_distances)-1] <= instant_stop_distance:
            return True
        elif (numpy.prod(self.recorded_distances)) != 0.0 and (sum(self.recorded_distances)/len(self.recorded_distances)) <= average_stop_distance: #calculating a the average over the last 3 distances
            return True
        else:
            return False
    
    def reset_echo_pins(self):
        GPIO.cleanup()
        self.TRIG_already_init=False
    
    @staticmethod
    def setmode():
        GPIO.setmode(GPIO.BCM)

    @classmethod
    def init_trigger_pin(cls):
        if not cls.TRIG_already_init:

            GPIO.setup(cls.TRIG, GPIO.OUT)
            time.sleep(0.001) 
            GPIO.output(cls.TRIG, False)
            time.sleep(0.001) 
            cls.TRIG_already_init=True
        
        


