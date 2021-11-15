#ifndef ECHO_DISTANZ_SENSOR
#define ECHO_DISTANZ_SENSOR

#include "Arduino.h"

class EchoSensor
{
  private:
  //declare needed variables
  int distance_mm = 0;
  int tof_time = 0;
  short int last_measurement = -1;
  short int trigPin, echoPin;

  public:
  //declare constructor and destructor
  EchoSensor(short int TRIG, short int ECHO);
  ~EchoSensor();

  //declare functions for class EchoSensor
  void setup_pins();
  int get_distance_in_mm();
  bool get_echo_dist_warning(short int minDist);
  
};

#endif
