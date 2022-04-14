#include "EchoSensor.h"

//implementation of EchoSensor class

EchoSensor::EchoSensor(short int TRIG, short int ECHO)
  : trigPin(TRIG), echoPin(ECHO)
{
  
}

EchoSensor::~EchoSensor()
{
  
}


//implementing functions of EchoSensor class

void EchoSensor::setup_pins()
{ 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  delay(5);
}


short EchoSensor::get_distance_in_mm()
{
  //calculate the max_time_for_measurement: t=(distance*2)/(343.5*10e⁻6)
  //unsigned int max_time_for_measurement = 4366; //this time equals a distance (from the sensor) of about 0.75 meter
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);


  tof_time = pulseIn(echoPin, HIGH, 3493); //4366 microseconds: this time equals a distance (from the sensor) of about 0.75 meter, 3493 equals about 0.6m
  
  //calculate distance, reminder: time needs to be divided by two because soundwave goes from sensor to object back to sensor
  //velocity of sound about 342,2 m/s or 0,3432 mm/microsecond
  //first check if measurement was within the time limit of 4366 microseconds (if not tof_time == 0)
  if(tof_time == 0){
    tof_time = 3493;
  }
  distance_mm = ((tof_time/2)*0.3432);
  
  return distance_mm;
  
}

bool EchoSensor::get_echo_dist_warning(short int minDist)
{
 
  short int measured_dist = get_distance_in_mm();
  
  bool is_warning_needed=(measured_dist <= minDist && last_measurement <= (minDist*1.3));

  if(measured_dist <= (minDist*0.8)){
    warning = true;
    return true;
  }
  
  if( is_warning_needed || warning){
    warning=false;
    if(is_warning_needed){
      warning = true;
    }
    
    last_measurement = measured_dist;
    return true;
  }
  else{
    warning = false;
    last_measurement = measured_dist;
    return false;
  }

    
}
