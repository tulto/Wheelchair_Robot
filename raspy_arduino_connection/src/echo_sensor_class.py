import RPi.GPIO as GPIO
import time
import numpy

class EchoSensor:

    cls.TRIG = 12

    def __init__(self, TRIGERpin, ECHOpin):
        #self.TRIG=TRIGERpin as test
        self.ECHO=ECHOpin
        self.recorded_distances=[0.0,0.0,0.0]

    def init_sensor(self):
        #GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        #GPIO.output(self.TRIG, False)
        time.sleep(0.1) 

    def get_distance_in_m(self):

        pulse_start = 0.0
        pulse_end = 0.0
        pulse_duration = 0.0

        GPIO.output(cls.TRIG,False)
        time.sleep(0.001)
        GPIO.output(cls.TRIG,True)
        time.sleep(0.00001)
        GPIO.output(cls.TRIG,False)

        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        temperature = 20.0 #(air)temperature in °C
        v_air = (331.5 +(0.6*temperature)) #speed of sound
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
    
    @staticmethod
    def setmode():
        GPIO.setmode(GPIO.BCM)

    @classmethod
    def init_trigger_pin():
        GPIO.setup(cls.TRIG, GPIO.OUT)
        GPIO.output(cls.TRIG, False)
        time.sleep(0.001) 
        
        


