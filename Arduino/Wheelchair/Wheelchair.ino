#include <ros.h>
#include "Motor_Controller.h"
#include "IMU.h"
#include <services_and_messages/Joystick.h>
#include "joystick.h"
#include "EchoSensor.h"
#include "ArduinoSTL.h"
#include "TOFLaserDistanzSensor.h"
#include "services_and_messages/TOF_sensor.h"
#include "services_and_messages/Echosensors.h"
#include "Filter_movement.h"

ros::NodeHandle nh;

//warning message for the TOF_Sensors (used for recognizing stairs)
//publisher for warning message
services_and_messages::TOF_sensor stair_warn_msg;
ros::Publisher stair_warning_pub("/stair_warning_dir", &stair_warn_msg);

//warning message for collision and
//publisher for collision warning message
services_and_messages::Echosensors collision_warn_msg;
ros::Publisher collision_warning_pub("/collision_warning_dir", &collision_warn_msg);

//TOF-IR sensors
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
std::vector<TOFLaserDistanzSensor*> tof_sensor_all = {&sensor4, &sensor3, &sensor2, &sensor1};

//function for init all tof-light-sensors with different i2c addresses (use this function in setup()
void initTOFirSetupPins() {
  
  // reset all sensors  
  for(int i = 0; i < tof_sensor_all.size(); i++){
    tof_sensor_all[i]->shutdown_sensor();
  }
  delay(10);


  // start all sensors
  for(int i = 0; i < tof_sensor_all.size(); i++){
    tof_sensor_all[i]->start_sensor();
  }
  delay(10);

  //giving all sensors new i2c addresses
  tof_sensor_all.pop_back();
  sensor1.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  sensor2.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  sensor3.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  sensor4.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  

  //fill sensor vector again with all pointers to sensor elements
  tof_sensor_all.push_back(&sensor1);
  tof_sensor_all.push_back(&sensor2);
  tof_sensor_all.push_back(&sensor3);
  tof_sensor_all.push_back(&sensor4); 
}

//function for generating the stair_warn_msg with corresponding direction of the warning
void send_stair_warning(TOFLaserDistanzSensor &front, TOFLaserDistanzSensor &left, TOFLaserDistanzSensor &right, TOFLaserDistanzSensor &back){

  stair_warn_msg.stair_warning_dir[0] = front.get_distance_warning(398, 490);
  stair_warn_msg.stair_warning_dir[1] = left.get_distance_warning(398, 490);
  stair_warn_msg.stair_warning_dir[2] = right.get_distance_warning(398, 490);
  stair_warn_msg.stair_warning_dir[3] = back.get_distance_warning(398, 490);

  //publish stair_warn_msg to ros
  stair_warning_pub.publish(&stair_warn_msg);
   
}

//Ultrasonic sensors
//define all the pins needed for the ultrasonicsensors
#define TRIG_PIN_ALL_SENSORS 44
#define ECHO_SENSOR_1_PIN 38 //echo-sensor-front
#define ECHO_SENSOR_2_PIN 27 //echo-sensor-left-front
#define ECHO_SENSOR_3_PIN 39 //echo-sensor-left-back
#define ECHO_SENSOR_4_PIN 49 //echo-sensor-right-front
#define ECHO_SENSOR_5_PIN 50 //echo-sensor-right-back
#define ECHO_SENSOR_6_PIN 40 //echo-sensor-back

//boolean values to only send collision_warn_msg every second code loop
bool send_collision_warn = false;
bool send_collision_warn_was_changed = false;

//implementing different EchoSensor objects
EchoSensor echo_sensor_1 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_1_PIN);
EchoSensor echo_sensor_2 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_2_PIN);
EchoSensor echo_sensor_3 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_3_PIN);
EchoSensor echo_sensor_4 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_4_PIN);
EchoSensor echo_sensor_5 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_5_PIN);
EchoSensor echo_sensor_6 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_6_PIN);

//put references (pointer to the objets) into a pointer vector for easier use
//std::vector<EchoSensor*> echo_all = {&echo_sensor_1, &echo_sensor_2, &echo_sensor_3, &echo_sensor_4, &echo_sensor_5, &echo_sensor_6};

//function for generating the collisono_warn_msg with corresponding direction of the warning
/*
void send_collision_warning(EchoSensor &front, EchoSensor &left_front, EchoSensor &left_back, EchoSensor &right_front, EchoSensor &right_back, EchoSensor &back){

  collision_warn_msg.echo_dir[1] = (left_front.get_echo_dist_warning(350) || left_back.get_echo_dist_warning(350));
  collision_warn_msg.echo_dir[0] = front.get_echo_dist_warning(450);
  collision_warn_msg.echo_dir[3] = back.get_echo_dist_warning(450);
  collision_warn_msg.echo_dir[2] = (right_front.get_echo_dist_warning(350) || right_back.get_echo_dist_warning(350));
  
  //publish the collision_warn_msg to ros
  collision_warning_pub.publish(&collision_warn_msg);
  
}*/

// Joystick
int x_movement = A1;
int y_movement = A2;
int t_movement = A3;
int button = 7;
int start = millis();
int timer = 100000;

services_and_messages::Joystick joy_msg;
ros::Publisher joystick("/movement/joystick", &joy_msg);


Motor_Controller drive;
IMU imu_;
Joystick joy(A1,A2,A3);
Filter_movement filter;

void setup() {
 Serial.begin(57600);  
 Serial1.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 Serial2.begin(115200);      // Roboteq SDC2130 COM (Must be 115200) 

 nh.initNode();
 drive.init(nh);
 imu_.init(nh);
 nh.advertise(joystick);

 nh.negotiateTopics();
 
 joy.init(nh);


 //TOF-IR-Sensors
 //declaring xshut pins for all tof-light-sensors to output pins
  for(int i = 0; i < tof_sensor_all.size(); i++){
    tof_sensor_all[i]->pin_setup();
  }
  
  Serial.println("All tof-ir-sensors shut down..");
  delay(10);
  Serial.println("Starting all tof-ir-sensors...");
  
  //setup sensors and i2c addresses of the different sensors
  initTOFirSetupPins();
  //advertise stair_warning_pub ros publisher
  nh.advertise(stair_warning_pub);
  nh.negotiateTopics();
  delay(2);

  
 //Echo-Sensors
 //setup all pins for the ultrasonic sensors
  /*for (int i = 0; i < echo_all.size(); i++) {
    echo_all[i]->setup_pins();
  }*/
  echo_sensor_1.setup_pins();
  echo_sensor_2.setup_pins();
  echo_sensor_3.setup_pins();
  echo_sensor_4.setup_pins();
  echo_sensor_5.setup_pins();
  echo_sensor_6.setup_pins();
  
  
  delay(10);
  //advertise collision_warning_pub ros publisher
  nh.advertise(collision_warning_pub);
  nh.negotiateTopics();
  delay(2);
  
 
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}


void loop() { 
  //abfragen des analogen Joystickes

  //reset send_collision_warn_was_changed to notice if the value has been changed in this iteration
  bool send_collision_warn_was_changed = false;
  /*
  joy_msg.x = analogRead(x_movement);
  joy_msg.y = analogRead(y_movement);
  joy_msg.t = analogRead(t_movement);
  joy_msg.button = digitalRead(button);
  joystick.publish( &joy_msg ); //senden der analogen Joystick Daten
  */
  
  //searching for possible collisions on the left side of the robot
  if(send_collision_warn){
    collision_warn_msg.echo_dir[1] = (echo_sensor_2.get_echo_dist_warning(350) || echo_sensor_3.get_echo_dist_warning(350));
  }
  //searching for possible collisions on the right side of the robot
  if(!send_collision_warn){
    collision_warn_msg.echo_dir[2] = (echo_sensor_4.get_echo_dist_warning(350) || echo_sensor_5.get_echo_dist_warning(350));
  }
  
  //tof-ir sensors for stair warning 
  //variable for checking if all tof_sensors have a message ready
  bool all_tof_sensors_data_ready = true;
  
  //start all tof_sensor measurements
  for(int i = 0; i < tof_sensor_all.size(); i++){
    tof_sensor_all[i]->start_single_measurement();
  }
  

  //check if measurement is ready 
  for(int i = 0; i<tof_sensor_all.size(); i++){
    if(!tof_sensor_all[i]->is_measurement_ready()){
      all_tof_sensors_data_ready = false;
    }
  }
  

  //if data is ready publish ros message
  if(all_tof_sensors_data_ready){
    
    send_stair_warning(sensor1, sensor2, sensor3, sensor4);

    all_tof_sensors_data_ready = false;

  }
  
  
  //ultrasonic sensors for collision warning
  //send_collision_warning(echo_sensor_1, echo_sensor_2, echo_sensor_3, echo_sensor_4, echo_sensor_5, echo_sensor_6);

  // filter.set_sensor(0,0,0,0);
  
  // query if joystick is used, if yes then it should predefine all movements
  // if there is movement from the joystick then use joystick - velocities else use sent movement from ROS 
  if (joy.movement()){   
    float vel[3];
    vel[0] = filter.blocking_path(-joy.x_velocity(), -joy.y_velocity(), -joy.t_velocity())[0];
    vel[1] = filter.blocking_path(-joy.x_velocity(), -joy.y_velocity(), -joy.t_velocity())[1];
    vel[2] = filter.blocking_path(-joy.x_velocity(), -joy.y_velocity(), -joy.t_velocity())[2];
    drive.set_movement(vel[0], vel[1], vel[2]);
  }else{
    drive.set_sent_movement();
    //drive.filter_movement();
  }
  
  //Set movement is executed
  drive.movement();
  
  //reset movement
  drive.set_movement(0, 0, 0);

  //sending encode values
  drive.send_encoder_count(timer);
  timer = millis()-start;
  start = millis();
  
  //IMU data will be send
  imu_.publish_imu_data(nh);
  imu_.publish_imu_cali();

  //searching for possible collisions on the front side of the robot
  if(send_collision_warn){
    collision_warn_msg.echo_dir[0] = echo_sensor_1.get_echo_dist_warning(450);
  }
  
  //searching for possible collisions on the back side of the robot
  if(!send_collision_warn){
    collision_warn_msg.echo_dir[3] = echo_sensor_6.get_echo_dist_warning(450);
  }


  if(!send_collision_warn){
    send_collision_warn=true;
    send_collision_warn_was_changed = true;
  }
   
  //publish the collision_warn_msg to ros
  if(send_collision_warn && !send_collision_warn_was_changed){
    send_collision_warn = false;
    collision_warning_pub.publish(&collision_warn_msg);

    filter.set_sensor(collision_warn_msg.echo_dir[0]||stair_warn_msg.stair_warning_dir[0], 
                    collision_warn_msg.echo_dir[1]||stair_warn_msg.stair_warning_dir[1], 
                    collision_warn_msg.echo_dir[2]||stair_warn_msg.stair_warning_dir[2], 
                    collision_warn_msg.echo_dir[3]||stair_warn_msg.stair_warning_dir[3]);
    
    collision_warn_msg.echo_dir[0]=true;
    collision_warn_msg.echo_dir[1]=true;
    collision_warn_msg.echo_dir[2]=true;
    collision_warn_msg.echo_dir[3]=true;
  }

  
  nh.spinOnce(); 
}
