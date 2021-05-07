#include <ros.h>
#include "Motor_Controller.h"

//Joystick
#define v 11 //front
#define r 10 //right
#define b 9  //back
#define l 8  //left


//encoder Position als Array
int32_t pos[4] = {0}; //[0] = links forne, [1] = rechst forne, [2] = links hinten, [3] = rechts hinten


Motor_Controller drive;


void setup() {
 Serial1.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 Serial2.begin(115200);      // Roboteq SDC2130 COM (Must be 115200) 

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
  //abfragen ob Joystik verwendet wird, wenn ja dann soll er alle Bewegungen vorgeben
  if (digitalRead(v) == 1 || digitalRead(r) == 1  || digitalRead(b) == 1  || digitalRead(l) == 1 ){
    float vel = 4;
    float x = (digitalRead(v)-digitalRead(b))*vel;
    float t = (digitalRead(l)-digitalRead(r))*vel;
    drive.set_movement(x, 0, t);
  }else {
    drive.set_sent_movement();
  }
  drive.movement();
  
  //Bewegung wird zurückgesetzt
  drive.set_movement(0, 0, 0);

  //Encoder Werte werden gesendet
  drive.send_encoder_count();
 
  delay(100); 
}
