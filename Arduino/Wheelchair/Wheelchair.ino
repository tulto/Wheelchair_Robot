#include <ros.h>
#include "Motor_Controller.h"
#include "IMU.h"
#include <services_and_messages/Joystick.h>
ros::NodeHandle nh;



//Joystick
#define v 11 //front
#define r 10 //right
#define b 9  //back
#define l 8  //left

int x_movement = A0;
int y_movement = A1;
int t_movement = A2;
int button = 7;
int start = micros();
int timer = 100000;

services_and_messages::Joystick joy_msg;
ros::Publisher joystick("/movement/joystick", &joy_msg);


Motor_Controller drive;
IMU imu_;

void setup() {
 Serial.begin(57600);  
 Serial1.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 Serial2.begin(115200);      // Roboteq SDC2130 COM (Must be 115200) 

 nh.initNode();
 drive.init(nh);
 imu_.init(nh);
 nh.advertise(joystick);
 
  
 //Joystick
 pinMode(v,INPUT);//front
 pinMode(r,INPUT);//right
 pinMode(b,INPUT);//back
 pinMode(l,INPUT);//left
 pinMode(button, INPUT);
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}


void loop() { 
  //abfragen des analogen Joystickes
  joy_msg.x = analogRead(x_movement);
  joy_msg.y = analogRead(y_movement);
  joy_msg.t = analogRead(t_movement);
  joy_msg.button = digitalRead(button);
  joystick.publish( &joy_msg ); //senden der analogen Joystick Daten
  

  //abfragen ob Joystik verwendet wird, wenn ja dann soll er alle Bewegungen vorgeben
  if (digitalRead(v) == 1 || digitalRead(r) == 1  || digitalRead(b) == 1  || digitalRead(l) == 1 ){
    float vel = 6;
    float x = (digitalRead(v)-digitalRead(b))*vel;
    float t = (digitalRead(l)-digitalRead(r))*vel;
    drive.set_movement(x, 0, t);
    drive.filter_movement();
  }else {
    drive.set_sent_movement();
    //drive.filter_movement();  
  }

  
  //gesetzte bewegungenen werden ausgeführt
  drive.movement();
  
  //Bewegung wird zurückgesetzt
  drive.set_movement(0, 0, 0);

  //Encoder Werte werden gesendet
  drive.send_encoder_count(timer);
  timer = micros()-start;
  start = micros();

  //IMU Werte und Kalibrierdaten werden gesendet
  imu_.publish_imu_data(nh);
  imu_.publish_imu_cali();

  nh.spinOnce();
 
  
}
