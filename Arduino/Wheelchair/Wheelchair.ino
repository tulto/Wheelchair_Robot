#include <ros.h>
#include "Motor_Controller.h"
#include "Echo_Sensor.h"
ros::NodeHandle nh;

#include <services_and_messages/Encoder.h>

//testzwecke Zeit pro durchlauf publishen
#include <std_msgs/Int64.h>
std_msgs::Int64 zeit_msg;
ros::Publisher dauer("/Zeit", &zeit_msg);


//Joystick
#define v 11 //front
#define r 10 //right
#define b 9  //back
#define l 8  //left


//encoder Position als Array
int32_t pos[4] = {0}; //[0] = links forne, [1] = rechst forne, [2] = links hinten, [3] = rechts hinten

long start;
long ende;

Motor_Controller drive;
Echo_Sensor sens;

void setup() {


 Serial1.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 Serial2.begin(115200);      // Roboteq SDC2130 COM (Must be 115200) 

 nh.initNode();
 drive.init(nh);
 sens.init(nh);

 //testzwecke zeitdauer pro durchlauf bestimmen
 nh.advertise(dauer);
 
  
 //Joystick
 pinMode(v,INPUT);//front
 pinMode(r,INPUT);//right
 pinMode(b,INPUT);//back
 pinMode(l,INPUT);//left
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}


void loop() { 
  
  
  start = millis();

  //sens.set_sensor(false, false, false, false);//[0] = front, [1] = links, [2] = rechts, [3] = hinten 

  //abfragen ob Joystik verwendet wird, wenn ja dann soll er alle Bewegungen vorgeben
  if (digitalRead(v) == 1 || digitalRead(r) == 1  || digitalRead(b) == 1  || digitalRead(l) == 1 ){
    float vel = 4;
    float x = (digitalRead(v)-digitalRead(b))*vel;
    float t = (digitalRead(l)-digitalRead(r))*vel;
    drive.set_movement(x, 0, t);
    //drive.filter_movement();
  }else {
    drive.set_sent_movement();
    //drive.filter_movement();
  }
  //gesetzte bewegungenen werden ausgeführt
  drive.movement();
  
  //Bewegung wird zurückgesetzt
  drive.set_movement(0, 0, 0);

  delay(0); //delay for dem Auslesen der Encoder Werte

  //Encoder Werte werden gesendet
  drive.send_encoder_count();

  //testzwecke für zeit pro durchlauf in millisekunden
  ende = millis();
  zeit_msg.data = ende - start;  
  dauer.publish(&zeit_msg);
  nh.spinOnce();
 
  
}
