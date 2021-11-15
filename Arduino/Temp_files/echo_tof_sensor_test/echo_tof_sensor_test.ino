
#include "EchoSensor.h"
#include "ArduinoSTL.h"
#include <ros.h>
#include "services_and_messages/Echosensors.h"

//define ROS NodeHandler
ros::NodeHandle nh;


//warning message for collision and
//publisher for collision warning message
services_and_messages::Echosensors collision_warn_msg;
ros::Publisher collision_warning_pub("/collision_warning_dir", &collision_warn_msg);


#define TRIG_PIN_ALL_SENSORS 5
#define ECHO_SENSOR_1_PIN 22
#define ECHO_SENSOR_2_PIN 9

EchoSensor echo_sensor_1 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_1_PIN);
EchoSensor echo_sensor_2 = EchoSensor(TRIG_PIN_ALL_SENSORS, ECHO_SENSOR_2_PIN);

//put a reference (a pointer to the objets) into a pointer vector for easier use
std::vector<EchoSensor*> echo_all = {&echo_sensor_1, &echo_sensor_2};


//function for generating the collisono_warn_msg with corresponding direction of the warning
void send_collision_warning(EchoSensor &front, EchoSensor &left /*, EchoSensor &right , EchoSensor &back*/){

  collision_warn_msg.echo_dir[0] = front.get_echo_dist_warning(450);
  collision_warn_msg.echo_dir[1] = left.get_echo_dist_warning(350);
  //collision_warn_msg.echo_dir[2] = right.get_echo_dist_warning(350);
  //collision_warn_msg.echo_dir[3] = back.get_echo_dist_warning(450);

  //just needed for now because the last sensors is not pressent
  collision_warn_msg.echo_dir[2] = false;
  collision_warn_msg.echo_dir[3] = false;

  collision_warning_pub.publish(&collision_warn_msg);
  
}


void setup() {
  Serial.begin(57600);

  while (! Serial) {
    delay(1);
  }

  nh.initNode();
  
  //Serial.print("Start programm...");

  //setup all pins for the echosensors
  for (int i = 0; i < echo_all.size(); i++) {
    echo_all[i]->setup_pins();
  }
  delay(10);

  nh.advertise(collision_warning_pub);

}

//variables for testing runtime
/*
unsigned long int lastTime = 0;
unsigned int delta = 0;
unsigned long int sum = 0;
unsigned int m = 0;
std::vector<int> dist = {};
*/

void loop() {
  //m++; //variable for runtime measurement

  /*//Printing different sensor Messages
  for (int i = 0; i < echo_all.size(); i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(echo_all[i]->get_distance_in_mm());
  }

  delay(100);*/

  send_collision_warning(echo_sensor_1, echo_sensor_2);
  delay(50);
  
  /*
    delta = millis()-lastTime;
    lastTime = millis();
    sum += delta;


    if(millis()>10000 & millis()<10500){
    Serial.print(sum/m);
    Serial.println("zeit");
    for(int i=0; i<dist.size(); i++){
      Serial.println(dist[i]);
    }

    }*/
    nh.spinOnce();

}
