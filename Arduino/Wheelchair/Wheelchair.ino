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
#define TOF_SENSOR_1_ADDRESS 0x31
#define TOF_SENSOR_2_ADDRESS 0x33
#define TOF_SENSOR_3_ADDRESS 0x35
#define TOF_SENSOR_4_ADDRESS 0x37
#define TOF_SENSOR_5_ADDRESS 0x39
#define TOF_SENSOR_6_ADDRESS 0x41
#define TOF_SENSOR_7_ADDRESS 0x42
#define TOF_SENSOR_8_ADDRESS 0x44

// define the xshut pins for the different sensors
#define XSHT_SENSOR_1 2 //front tof sensor
#define XSHT_SENSOR_2 4 //left tof sensor
#define XSHT_SENSOR_3 5 //right tof sensor
#define XSHT_SENSOR_4 3 //back tof sensor
#define XSHT_SENSOR_5 33 //left front tof sensor
#define XSHT_SENSOR_6 53 //right front tof sensor
#define XSHT_SENSOR_7 35 //right back tof sensor
#define XSHT_SENSOR_8 51 //left backtof sensor

//implementing different TOFLaserDistanzSensor objects
TOFLaserDistanzSensor sensor1(TOF_SENSOR_1_ADDRESS, XSHT_SENSOR_1);  //front
TOFLaserDistanzSensor sensor2(TOF_SENSOR_2_ADDRESS, XSHT_SENSOR_2);  //left
TOFLaserDistanzSensor sensor3(TOF_SENSOR_3_ADDRESS, XSHT_SENSOR_3);  //right
TOFLaserDistanzSensor sensor4(TOF_SENSOR_4_ADDRESS, XSHT_SENSOR_4);  //back
TOFLaserDistanzSensor sensor5(TOF_SENSOR_5_ADDRESS, XSHT_SENSOR_5);  //left front
TOFLaserDistanzSensor sensor6(TOF_SENSOR_6_ADDRESS, XSHT_SENSOR_6);  //right front
TOFLaserDistanzSensor sensor7(TOF_SENSOR_7_ADDRESS, XSHT_SENSOR_7);  //right back
TOFLaserDistanzSensor sensor8(TOF_SENSOR_8_ADDRESS, XSHT_SENSOR_8);  //left back

//variable to check if the tof_sensors had an issue
bool tof_dead = false;

//vector of pointers to TOFLaserDistanzSensor objects
std::vector<TOFLaserDistanzSensor*> tof_sensor_all = {&sensor8, &sensor7, &sensor6, &sensor5, &sensor4, &sensor3, &sensor2, &sensor1};

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
  sensor5.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  sensor6.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  sensor7.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  sensor8.set_i2c_address(tof_sensor_all);
  tof_sensor_all.pop_back();
  
  

  tof_sensor_all = {&sensor8, &sensor7, &sensor6, &sensor5, &sensor4, &sensor3, &sensor2, &sensor1};
}

//function for generating the stair_warn_msg with corresponding direction of the warning
void send_stair_warning(TOFLaserDistanzSensor &front, TOFLaserDistanzSensor &left, TOFLaserDistanzSensor &right, TOFLaserDistanzSensor &back, TOFLaserDistanzSensor &left_front, TOFLaserDistanzSensor &right_front, TOFLaserDistanzSensor &right_back, TOFLaserDistanzSensor &left_back){

  bool left_front_val = left_front.get_distance_warning(180, 540);
  bool right_front_val = right_front.get_distance_warning(180, 540);
  bool right_back_val = right_back.get_distance_warning(180, 540);
  bool left_back_val = left_back.get_distance_warning(180, 540);
  
  stair_warn_msg.stair_warning_dir[0] = front.get_distance_warning(250, 560) || left_front_val || right_front_val;
  stair_warn_msg.stair_warning_dir[1] = left.get_distance_warning(250, 560) || left_front_val || left_back_val;
  stair_warn_msg.stair_warning_dir[2] = right.get_distance_warning(250, 560) || right_front_val || right_back_val;
  stair_warn_msg.stair_warning_dir[3] = back.get_distance_warning(250, 560) || left_back_val || right_back_val;


  //publish stair_warn_msg to ros
  stair_warning_pub.publish(&stair_warn_msg);
   
}

//Ultrasonic sensors
//define all the pins needed for the ultrasonicsensors
#define TRIG_PIN_FRONT 11       //trig-sensor-front
#define TRIG_PIN_LEFT_FRONT 10  //trig-sensor-left-front
#define TRIG_PIN_LEFT_BACK 12   //trig-sensor-left-back
#define TRIG_PIN_RIGHT_FRONT 8  //trig-sensor-right-front
#define TRIG_PIN_RIGHT_BACK 9   //trig-sensor-right-back
#define TRIG_PIN_BACK 13        //trig-sensor-back

#define ECHO_SENSOR_1_PIN 38 //echo-sensor-front
#define ECHO_SENSOR_2_PIN 27 //echo-sensor-left-front
#define ECHO_SENSOR_3_PIN 39 //echo-sensor-left-back
#define ECHO_SENSOR_4_PIN 49 //echo-sensor-right-front
#define ECHO_SENSOR_5_PIN 32 //echo-sensor-right-back
#define ECHO_SENSOR_6_PIN 40 //echo-sensor-back

//boolean values to only send collision_warn_msg every second code loop
bool send_collision_warn = false;
bool send_collision_warn_was_changed = false;
//measurement of the ultrasonic sensors takes 2 iterations of the code (for better measurements)
  bool echo_front = true;
  bool echo_left = true;
  bool echo_right = true;
  bool echo_back = true;

//implementing different EchoSensor objects
EchoSensor echo_sensor_1 = EchoSensor(TRIG_PIN_FRONT, ECHO_SENSOR_1_PIN);
EchoSensor echo_sensor_2 = EchoSensor(TRIG_PIN_LEFT_FRONT, ECHO_SENSOR_2_PIN);
EchoSensor echo_sensor_3 = EchoSensor(TRIG_PIN_LEFT_BACK, ECHO_SENSOR_3_PIN);
EchoSensor echo_sensor_4 = EchoSensor(TRIG_PIN_RIGHT_FRONT, ECHO_SENSOR_4_PIN);
EchoSensor echo_sensor_5 = EchoSensor(TRIG_PIN_RIGHT_BACK, ECHO_SENSOR_5_PIN);
EchoSensor echo_sensor_6 = EchoSensor(TRIG_PIN_BACK, ECHO_SENSOR_6_PIN);

//put references (pointer to the objets) into a pointer vector for easier use
//std::vector<EchoSensor*> echo_all = {&echo_sensor_1, &echo_sensor_2, &echo_sensor_3, &echo_sensor_4, &echo_sensor_5, &echo_sensor_6};

// Joystick
int x_movement = A1;
int y_movement = A2;
int t_movement = A3;
int button = 7;
unsigned long start = millis();
int timer = 1000;

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
  delay(5);
  //setup sensors and i2c addresses of the different sensors
  initTOFirSetupPins();
  delay(2);

  //advertise stair_warning_pub ros publisher
  nh.advertise(stair_warning_pub);
  nh.negotiateTopics();
  delay(2);

  
 //Echo-Sensors
 
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
  
  //variable for checking if all tof_sensors have a message ready
  bool all_tof_sensors_data_ready = true;

  //following variables are for i2c scanner to check if we have an address of 0x29
  //which would mean that the tof-sensors-died
  byte error;
  byte address=41;
  
  //searching for possible collisions on the front side of the robot
  if(send_collision_warn){
    collision_warn_msg.echo_dir[0] = echo_sensor_1.get_echo_dist_warning(390);
  }
  //searching for possible collisions on the right-front and left-back
  if(!send_collision_warn){
    collision_warn_msg.echo_dir[2] = echo_sensor_4.get_echo_dist_warning(250);
    collision_warn_msg.echo_dir[1] = echo_sensor_3.get_echo_dist_warning(250);
  }


  //tof-ir sensors for stair warning   
  //start all tof_sensor measurements (only needed when single and not continious measurement is set)
  for(int i = 0; i < tof_sensor_all.size(); i++){
    tof_sensor_all[i]->start_single_measurement();
  }
  
  
  //check if measurement is ready 
  for(int i = 0; i<tof_sensor_all.size(); i++){
    if(!tof_sensor_all[i]->is_measurement_ready()){
      all_tof_sensors_data_ready = false;
      break;
    }
  }
  
  //if data is ready publish ros stair_warn_dir message
  if(all_tof_sensors_data_ready){
    send_stair_warning(sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, sensor8);
    for(int i = 0; i < tof_sensor_all.size(); i++){
      if((tof_sensor_all[i]->get_measured_dist() == 0)){
        tof_dead = true;
        filter.set_sensor(1,1,1,1);
      }
    }
    all_tof_sensors_data_ready = false;
  } 

  //the following code checks if one i2c addres is 0x29
  //this would mean that the tof sensors had an issue and need to be restarted
  Wire.beginTransmission(address); //check if device has address 0x29, this would be an error
  error = Wire.endTransmission();
  if (error == 0)
  {
    tof_dead = true;
    filter.set_sensor(1,1,1,1);
  }
  
  //filter.set_sensor(0,0,0,0);
  
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

  // overwrites the movement filter
  if (joy.pressed_button(2000)){
    drive.set_movement(-joy.x_velocity(), -joy.y_velocity(), -joy.t_velocity());
  }
  
  //Set movement is executed
  drive.movement();
  
  //reset movement
  drive.set_movement(0, 0, 0);

  //sending encode values
  timer = millis()-start;
  start = millis();
  drive.send_encoder_count(timer);
  
  
  //IMU data will be send
  imu_.publish_imu_data(nh);
  imu_.publish_imu_cali();

  //searching for possible collisions on the left-front and right-back
  if(send_collision_warn){
    collision_warn_msg.echo_dir[1] = (echo_sensor_2.get_echo_dist_warning(250) || collision_warn_msg.echo_dir[1]);
    collision_warn_msg.echo_dir[2] = (echo_sensor_5.get_echo_dist_warning(250) || collision_warn_msg.echo_dir[2]);
  }
  
  //searching for possible collisions on the back side of the robot
  if(!send_collision_warn){
    collision_warn_msg.echo_dir[3] = echo_sensor_6.get_echo_dist_warning(405);
  }


  if(!send_collision_warn){
    send_collision_warn=true;
    send_collision_warn_was_changed = true;
  }
   
  //publish the collision_warn_msg to ros
  if(send_collision_warn && !send_collision_warn_was_changed){
    send_collision_warn = false;
    collision_warning_pub.publish(&collision_warn_msg);

    echo_front = collision_warn_msg.echo_dir[0];
    echo_left = collision_warn_msg.echo_dir[1];
    echo_right = collision_warn_msg.echo_dir[2];
    echo_back = collision_warn_msg.echo_dir[3];
    
  }
  
  filter.set_sensor(echo_front || stair_warn_msg.stair_warning_dir[0], 
                    echo_left || stair_warn_msg.stair_warning_dir[1], 
                    echo_right || stair_warn_msg.stair_warning_dir[2], 
                    echo_back || stair_warn_msg.stair_warning_dir[3]);
  if(tof_dead){
    for(int i = 0; i < tof_sensor_all.size(); i++){
        tof_sensor_all[i]->shutdown_sensor();
        delay(2);
    }
    initTOFirSetupPins();
    delay(2);
    tof_dead=false;
  }
  
  nh.spinOnce(); 
}
