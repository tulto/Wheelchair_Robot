#include <ros.h>
#include "Motor_Controller.h"
#include <std_msgs/Int64.h>


#define ENCA 2 
#define ENCB 3 


// for testing purpose publish encoder value to /encoder only one encoder
std_msgs::Int64 int_msg;
ros::Publisher encoder("/encoder", &int_msg); 

ros::NodeHandle nh;

int pos = 0;


void setup() {
 Serial.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)

 pinMode(ENCA,INPUT);
 pinMode(ENCB,INPUT);
 attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
 nh.initNode();
 nh.advertise(encoder);
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}


void loop() { 
  //Motor_Control test;
  //test.moveset_normal();
  Motor_Controller test;
  test.movement();

  //publishing encoder value to /encoder
  int_msg.data = pos;
  encoder.publish( &int_msg); 
  nh.spinOnce();
  delay(100);
}


void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}















//Testing Purpose
//as a normal driver it ist possible to turn an Drive simultanious
void move_normal (int velocity, int turning){
    
  motor_controller_front(1,velocity + turning);
  motor_controller_front(2,-velocity + turning);
  motor_controller_back(1,velocity + turning);
  motor_controller_back(2,-velocity + turning);
  delay(100);
}

void move_special (int x, int y){ //depending on x and y the wheelchair should drive in a drifferent angle
  motor_controller_front(1, -x);
  motor_controller_front(2, y);
  motor_controller_back(1, y);
  motor_controller_back(2, -x);
  delay(100);
}

void motor_controller_front (int chanel, int velocity){
  Serial1.print("!G ");
  Serial1.print(chanel);
  Serial1.print(" ");
  Serial1.println(velocity);
}

void motor_controller_back (int chanel, int velocity){
  Serial2.print("!G ");
  Serial2.print(chanel);
  Serial2.print(" ");
  Serial2.println(velocity);
}
