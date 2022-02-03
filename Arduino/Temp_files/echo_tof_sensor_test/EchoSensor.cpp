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


int EchoSensor::get_distance_in_mm()
{
  //calculate the max_time_for_measurement: t=(distance*2)/(343.5*10e⁻6)
  unsigned int max_time_for_measurement = 5827; //this time equals a distance (from the sensor) of about 1.0 meter
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long int measurement_start_time = micros();
  
  while(!digitalRead(echoPin)){
    if((micros() - measurement_start_time) > 550){
      tof_time = -10;
      break;
    }
  }

  measurement_start_time = micros();
  
  while(digitalRead(echoPin)){
    //Serial.print("in");
    tof_time = (micros()-measurement_start_time);
    
    if(tof_time < 0){
      tof_time = (4294967295 - measurement_start_time + micros());
    }
    
    if(tof_time > max_time_for_measurement){
      break;
    }
  }
  
  //calculate distance, reminder: time needs to be divided by two because soundwave goes from sensor to object back to sensor
  //velocity of sound about 342,2 m/s or 0,3432 mm/microsecond
  distance_mm = ((tof_time/2)*0.3432);
  
  return distance_mm;
  
}

bool EchoSensor::get_echo_dist_warning(short int minDist)
{
  short int measured_dist = get_distance_in_mm();
  if(measured_dist <= minDist || last_measurement <= minDist){
    last_measurement = measured_dist;
    return true;
  }
  else{
    last_measurement = measured_dist;
    return false;
  }
}
