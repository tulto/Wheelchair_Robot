import RPi.GPIO as GPIO
import time
import numpy

class EchoSensor:

    def __init__(self, TRIGERpin, ECHOpin):
        self.TRIG=TRIGERpin
        self.ECHO=ECHOpin
        self.recorded_distances=[0.0,0.0,0.0]

    def init_sensor(self):
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.output(self.TRIG, False)
        time.sleep(0.1) 

    def get_distance_in_m(self):

        pulse_start = 0.0
        pulse_end = 0.0
        pulse_duration = 0.0

        GPIO.output(self.TRIG,True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG,False)

        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_start - pulse_end
        temperature = 20.0 #(air)temperature in °C
        v_air = (331.5 +(0.6*temperature)) #speed of sound
        distance = round(((v_air/2)*pulse_duration), 4) #divide v_air by 2 because wave travels two time the distance
        self.recorded_distances.append(distance)
        self.recorded_distances.pop(0)
        return distance

    def distance_warning(self):
        if (sum(self.recorded_distances)) != 0.0 and self.recorded_distances[len(self.recorded_distances)-1] <= 0.25:
            return True
        elif (numpy.prod(self.recorded_distances)) != 0.0 and (sum(self.recorded_distances)/len(self.recorded_distances)) <= 0.30: #calculating a the average over the last 3 distances
            return True
        else:
            return False
    
    def reset_echo_pins(self):
        GPIO.cleanup()




