#include <ros.h>
#include "Motor_Controller.h"
#include <geometry_msgs/Twist.h>
#include <services_and_messages/Encoder.h>

//encoder
#define lf 2  //left front  1
#define lb 3  //left back   2
#define rf 18 //right front 3
#define rb 19 //right back  4

//Joystick
#define v 8
#define r 9
#define h 10
#define l 11


// for testing purpose publish encoder value to /encoder only one encoder
services_and_messages::Encoder encoder_msg;
ros::Publisher encoder("/encoder", &encoder_msg); 

ros::NodeHandle nh;

//encoder Position als Array
int32_t pos[4] = {0}; //[0] = links forne, [1] = rechst forne, [2] = links hinten, [3] = rechts hinten


Motor_Controller drive;


void setup() {
 Serial1.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 Serial2.begin(115200);      // Roboteq SDC2130 COM (Must be 115200) 

 //jeder Block ist für einen Motorencoder zuständig
 pinMode(lf,INPUT); //links vorne
 pinMode(4,INPUT);
 
 pinMode(lb,INPUT); //links hinten
 pinMode(5,INPUT);
 
 pinMode(rf,INPUT); //rechts vorne
 pinMode(17,INPUT);
 
 pinMode(rb,INPUT); //rechts hinten
 pinMode(20,INPUT);
 
 attachInterrupt(digitalPinToInterrupt(lf),readEncoder_lf,RISING);
 attachInterrupt(digitalPinToInterrupt(lb),readEncoder_lb,RISING);
 attachInterrupt(digitalPinToInterrupt(rf),readEncoder_rf,RISING);
 attachInterrupt(digitalPinToInterrupt(rb),readEncoder_rb,RISING);
 nh.initNode();
 nh.advertise(encoder);

 //Joystick
 pinMode(v,INPUT);//front
 pinMode(r,INPUT);//right
 pinMode(h,INPUT);//back
 pinMode(l,INPUT);//left
 
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}


void loop() { 
  //abfragen ob Joystik verwendet wird, wenn ja dann soll er alle Bewegungen vorgeben
  if (digitalRead(v) == 1 || digitalRead(r) == 1  || digitalRead(h) == 1  || digitalRead(l) == 1 ){
    float vel = 4;
    float x = (digitalRead(l)-digitalRead(r))*vel;
    float t = (digitalRead(v)-digitalRead(h))*vel;
    drive.set_movement(x, 0, t);
  }else {
    drive.set_sent_movement();
  }
  drive.movement();
  
  //Bewegung wird zurückgesetzt
  drive.set_movement(0, 0, 0);

  //publishing encoder value to /encoder
  for (int i = 0; i<4; i++){ //array muss überführt werden
    encoder_msg.encoder_wheel[i] = pos[i]; //[0] = links forne, [1] = rechst forne, [2] = links hinten, [3] = rechts hinten
  }
  encoder.publish( &encoder_msg); 
  nh.spinOnce();  
  delay(100);
}





//every encoder inside motor gets function
void readEncoder_lf(){ //left front pos[0]
  int b = digitalRead(4);
  if(b > 0){
    pos[0]++;
  }
  else{
    pos[0]--;
  }
}
void readEncoder_rf(){ //right front pos[1]
  int b = digitalRead(17);
  if(b > 0){
    pos[1]++;
  }
  else{
    pos[1]--;
  }
}
void readEncoder_lb(){ //left back pos[2]
  int b = digitalRead(5);
  if(b > 0){
    pos[2]++;
  }
  else{
    pos[2]--;
  }
}
void readEncoder_rb(){ //right back[3]
  int b = digitalRead(20);
  if(b > 0){
    pos[3]++;
  }
  else{
    pos[3]--;
  }
}
