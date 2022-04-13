#ifndef TOF_LASER_DISTANZ_SENSOR
#define TOF_LASER_DISTANZ_SENSOR

#include <Wire.h>
#include "VL53L0X.h"
#include "ArduinoSTL.h"

class TOFLaserDistanzSensor
{
  private:
    VL53L0X tof_sensor = VL53L0X();
    byte i2c_address;
    byte xshut_pin;
    uint16_t measured_dist = 0;
    short last_measurement = -1;
    bool measurement_in_progress = false;
    
  public: 
    //declaring constructor and destructor
    TOFLaserDistanzSensor(short int i2cAddress, short int xshutPin);
    ~TOFLaserDistanzSensor();

    //declaring needed functions
      //For using more than one VL53L0X sensor you need to shut down all sensors which not yet
      //have changed addresses. Therefore we give a vector with all the sensors who need to be shut down
      //to the set_i2c_address function
    void start_sensor();
    void shutdown_sensor();
    void pin_setup();
    void set_i2c_address(std::vector<TOFLaserDistanzSensor*> sensorsToShutDown);
    uint16_t distance_measurement_mm_whole();
    void start_single_measurement();
    void start_continuous_measurement(uint32_t period_for_measuring = 0);
    bool is_measurement_ready();
    uint16_t get_distance_mm();
    uint16_t get_continuous_distance_mm();
    bool get_distance_warning(short minDist, short maxDist);
    bool get_distance_warning_continuous(short minDist, short maxDist);
    uint16_t get_measured_dist();
  
};


#endif
