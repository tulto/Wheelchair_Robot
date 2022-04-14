#include "TOFLaserDistanzSensor.h"

//implementing Constructor for TOFLaserDistanzSensor class
TOFLaserDistanzSensor::TOFLaserDistanzSensor(short int i2cAddress, short int xshutPin)
  : i2c_address(i2cAddress), xshut_pin(xshutPin)
{
  
}


//implementing the destructor
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
    delay(3);
  }

  start_sensor();
  delay(5);

  tof_sensor.setAddress(0x29);
  delay(3);
  
  tof_sensor.setTimeout(50); //set time out after which a distance reading will be aborted in ms
  
  if (!tof_sensor.init())
  {
    Serial.println("Failed to initialize sensor!");
    //while (1) {}
  }
  
  tof_sensor.setAddress(i2c_address);
  
  delay(3);
  
  tof_sensor.setMeasurementTimingBudget(20000); //reduce timing budget to 20ms because high accuracy is
                                                //not needed in our application but runtime is very important

  delay(3);
  
}


uint16_t TOFLaserDistanzSensor::distance_measurement_mm_whole()
{
  uint16_t distance = tof_sensor.readRangeSingleMillimeters();

  if(!tof_sensor.timeoutOccurred()) //if a timeout occured give back 0 (
  {
    return distance;
  }
  else
  {
    return 0;
  }
}


void TOFLaserDistanzSensor::start_single_measurement()
{
  if(!measurement_in_progress){
    measurement_in_progress = true;
    tof_sensor.startRangeSingleMillimeters();
  }
}


void TOFLaserDistanzSensor::start_continuous_measurement(uint32_t period_for_measuring = 0){
  tof_sensor.startContinuous(period_for_measuring);
}

bool TOFLaserDistanzSensor::is_measurement_ready()
{
  return (tof_sensor.isSingleRangeMeasurementReady());
}


uint16_t TOFLaserDistanzSensor::get_distance_mm()
{
  measurement_in_progress = false;
  return (tof_sensor.readSingleRangeReadyMeasurementMillimeters());
}

uint16_t TOFLaserDistanzSensor::get_continuous_distance_mm(){
  return(tof_sensor.readRangeContinuousMillimeters());
}

bool TOFLaserDistanzSensor::get_distance_warning(short minDist, short maxDist)
{
  measured_dist = get_distance_mm();
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

bool TOFLaserDistanzSensor::get_distance_warning_continuous(short minDist, short maxDist)
{
  measured_dist = get_continuous_distance_mm();
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

uint16_t TOFLaserDistanzSensor::get_measured_dist(){
  return measured_dist;
}
