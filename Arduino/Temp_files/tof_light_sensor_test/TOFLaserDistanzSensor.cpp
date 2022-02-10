#include "TOFLaserDistanzSensor.h"

//implementing Constructor for TOFLaserDistanzSensor class
TOFLaserDistanzSensor::TOFLaserDistanzSensor(short int i2cAddress, short int xshutPin)
  : i2c_address(i2cAddress), xshut_pin(xshutPin)
{
  
}


//implementing the deconstructor
TOFLaserDistanzSensor::~TOFLaserDistanzSensor()
{
  
}

//implementing all methods of TOFLaserDistanzSensor class

void TOFLaserDistanzSensor::pin_setup()
{
  pinMode(xshut_pin, OUTPUT);
  digitalWrite(xshut_pin, LOW);
}


void TOFLaserDistanzSensor::start_sensor()
{
  digitalWrite(xshut_pin, HIGH);
}


void TOFLaserDistanzSensor::shutdown_sensor()
{
  digitalWrite(xshut_pin, LOW);
}


void TOFLaserDistanzSensor::set_i2c_address(std::vector<TOFLaserDistanzSensor*> sensorsToShutDown)
{
  for(int i = 0; i < sensorsToShutDown.size(); i++)
  {
    sensorsToShutDown[i]->shutdown_sensor();
  }

  start_sensor();
  delay(10);
  
  if(!tof_sensor.begin(i2c_address)) {
    Serial.println(("Failed to boot VL53L0X"));
    while(1);
  }
  
  delay(10);
  
}


int TOFLaserDistanzSensor::distance_measurement_mm_whole()
{
  tof_sensor.rangingTest(&measurements, false);

  if(measurements.RangeStatus != 4) //if measurement is not out of range
  {
    return measurements.RangeMilliMeter;
  }
  else
  {
    return -1;
  }
}


void TOFLaserDistanzSensor::start_single_measurement()
{
  if(!measurement_in_progress){
    measurement_in_progress = true;
    tof_sensor.startRange();
  }
}


bool TOFLaserDistanzSensor::is_measurement_ready()
{
  return (tof_sensor.isRangeComplete());
}


int TOFLaserDistanzSensor::get_distance_mm()
{
  measurement_in_progress = false;
  return (tof_sensor.readRangeResult());
}

bool TOFLaserDistanzSensor::get_distance_warning(int minDist, int maxDist)
{
  short int measured_dist = get_distance_mm();
  if(measured_dist >= maxDist || measured_dist <= minDist || last_measurement >= maxDist || last_measurement <= minDist ){
    /*if(measured_dist >= 1000){
      return false;
    }*/
    last_measurement = measured_dist;
    return true;
  }
  else{
    last_measurement = measured_dist;
    return false;
  }
}
