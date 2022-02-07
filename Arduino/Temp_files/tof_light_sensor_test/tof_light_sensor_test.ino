
#include "TOFLaserDistanzSensor.h"
#include <ros.h>
#include "services_and_messages/TOF_sensor.h"

//define ROS NodeHandler
ros::NodeHandle nh;


//warning message for the TOF_Sensors (used for recognizing stairs)
//publisher for warning message
services_and_messages::TOF_sensor stair_warn_msg;
ros::Publisher stair_warning_pub("/stair_warning_dir", &stair_warn_msg);


// defining the addresses of the different sensors
#define TOF_SENSOR_1_ADDRESS 0x30
#define TOF_SENSOR_2_ADDRESS 0x31
#define TOF_SENSOR_3_ADDRESS 0x32
#define TOF_SENSOR_4_ADDRESS 0x33


// define the xshut pins for the different sensors
#define XSHT_SENSOR_1 2 //front tof sensor
#define XSHT_SENSOR_2 4 //left tof sensor
#define XSHT_SENSOR_3 5 //right tof sensor
#define XSHT_SENSOR_4 3 //back tof sensor


//implementing different TOFLaserDistanzSensor objects
TOFLaserDistanzSensor sensor1(TOF_SENSOR_1_ADDRESS, XSHT_SENSOR_1);
TOFLaserDistanzSensor sensor2(TOF_SENSOR_2_ADDRESS, XSHT_SENSOR_2);
TOFLaserDistanzSensor sensor3(TOF_SENSOR_3_ADDRESS, XSHT_SENSOR_3);
TOFLaserDistanzSensor sensor4(TOF_SENSOR_4_ADDRESS, XSHT_SENSOR_4);

//vector of pointers to TOFLaserDistanzSensor objects
std::vector<TOFLaserDistanzSensor*> sensors_all = {&sensor4, &sensor3, &sensor2, &sensor1};

//function for init all sensors with different i2c addresses (use this function in setup()
void initSetupPins() {
  
  // reset all sensors  
  for(int i = 0; i < sensors_all.size(); i++){
    sensors_all[i]->shutdown_sensor();
  }
  delay(10);


  // start all sensors
  for(int i = 0; i < sensors_all.size(); i++){
    sensors_all[i]->start_sensor();
  }
  delay(10);

  //giving all sensors new i2c addresses
  sensors_all.pop_back();
  sensor1.set_i2c_address(sensors_all);
  sensors_all.pop_back();
  sensor2.set_i2c_address(sensors_all);
  sensors_all.pop_back();
  sensor3.set_i2c_address(sensors_all);
  sensors_all.pop_back();
  sensor4.set_i2c_address(sensors_all);
  sensors_all.pop_back();

  //fill sensor vector again with all pointers to sensor elements
  sensors_all.push_back(&sensor1);
  sensors_all.push_back(&sensor2);
  sensors_all.push_back(&sensor3);
  sensors_all.push_back(&sensor4); 
  
}


//function for generating the stair_warn_msg with corresponding direction of the warning
void send_stair_warning(TOFLaserDistanzSensor &front, TOFLaserDistanzSensor &left, TOFLaserDistanzSensor &right, TOFLaserDistanzSensor &back){

  stair_warn_msg.stair_warning_dir[0] = front.get_distance_warning(398, 470);
  stair_warn_msg.stair_warning_dir[1] = left.get_distance_warning(398, 470);
  stair_warn_msg.stair_warning_dir[2] = right.get_distance_warning(398, 470);
  stair_warn_msg.stair_warning_dir[3] = back.get_distance_warning(398, 470);

  //publish stair_warn_msg to ros
  stair_warning_pub.publish(&stair_warn_msg);
   
}


//----------------------setup----------------------------------
void setup() {
  Serial.begin(57600);

  while (! Serial) { delay(1); }
  nh.initNode();  
  
  //declaring xshut pins for all sensors to output pins
  for(int i = 0; i < sensors_all.size(); i++){
    sensors_all[i]->pin_setup();
  }

  Serial.println("All sensors shut down..");

  delay(10);

  Serial.println("Starting all Sensors...");
  
  //setup sensors and i2c addresses of the different sensors
  initSetupPins();

  nh.advertise(stair_warning_pub);
 
}

//variables for testing runtime
/*
long int lastTime = 0;
long int nowTime;
int delta;
int sum = 0;
long int m = 0;
std::vector<int> measured_values = {};
*/

//-------------------------loop---------------------------
//NOTE: everything can be done without the need of the sensors_all vector, but not as elegant
void loop() {
  //m++; //just needed for runtime check

  //variable for checking if all tof_sensors have a message ready
  bool all_tof_sensors_data_ready = true;
  
  //start all sensor measurements
  for(int i = 0; i < sensors_all.size(); i++){
    sensors_all[i]->start_single_measurement();
  }

  //check if measurement is ready for all sensors and print output
  for(int i = 0; i<sensors_all.size(); i++){
    if(!sensors_all[i]->is_measurement_ready()){
      all_tof_sensors_data_ready = false;
    }
  }

  //if data ready publish ros message
  if(all_tof_sensors_data_ready){
    /*
    for(int i = 0; i < sensors_all.size(); i++){
        measured_values.push_back(sensors_all[i]->get_distance_mm());
    } */

    for(int i = 0; i<sensors_all.size(); i++){
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(sensors_all[i]->get_distance_mm());
    } 
    
    send_stair_warning(sensor1, sensor2, sensor3, sensor4);
    
    all_tof_sensors_data_ready = false;

  }

  delay(750);

  //code for testing needed runtime
  /*
  nowTime=millis();
  delta = nowTime-lastTime;
  sum = sum + delta;
  lastTime = nowTime;
  if(nowTime > 10000 && nowTime <10040){
    for(int i = 0; i<measured_values.size(); i++){
      Serial.print(measured_values[i]);
      Serial.print(" , ");
    }
    Serial.println();
    Serial.print("Anzahl aufgenommener Werte: ");
    Serial.println(measured_values.size());
    Serial.print("\n");
    Serial.println(m);
    Serial.print("\n");
    Serial.println(sum/m);
    while(1);
  }*/

  nh.spinOnce();
  
  
}
