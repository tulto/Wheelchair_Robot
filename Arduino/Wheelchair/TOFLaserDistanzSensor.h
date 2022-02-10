#ifndef TOF_LASER_DISTANZ_SENSOR
#define TOF_LASER_DISTANZ_SENSOR

#include "Adafruit_VL53L0X_short.h"
#include "ArduinoSTL.h"

class TOFLaserDistanzSensor
{
  private:
    //declaring private variables
    VL53L0X_RangingMeasurementData_t measurements;
    short int i2c_address;
    short int xshut_pin;
    short int last_measurement = -1;
    bool measurement_in_progress = false;
    Adafruit_VL53L0X_short tof_sensor = Adafruit_VL53L0X_short();

  public: 
    //declaring constructor and destructor
    TOFLaserDistanzSensor(short int i2cAddress, short int xshutPin);
    ~TOFLaserDistanzSensor();

    //declaring needed functions
      //For using more than one vl53l0x sensor you need to shut down all sensors which not yet
      //have changed addresses. Therefore we give a vector with all the sensors who need to be shut down
      //to the set_i2c_address function
    void start_sensor();
    void shutdown_sensor();
    void pin_setup();
    void set_i2c_address(std::vector<TOFLaserDistanzSensor*> sensorsToShutDown);
    int distance_measurement_mm_whole();
    void start_single_measurement();
    bool is_measurement_ready();
    int get_distance_mm();
    bool get_distance_warning(int minDist, int maxDist);
  
};


#endif
