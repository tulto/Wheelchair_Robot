#include <ros.h>
#include "Motor_Controller.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>


#define lf 2  //left front  1
#define lb 3  //left back   2
#define rf 18 //right front 3
#define rb 19 //right back  4


// for testing purpose publish encoder value to /encoder only one encoder
std_msgs::Int64 int_msg;
ros::Publisher encoder("/encoder", &int_msg); 

ros::NodeHandle nh;

int pos_lf = 0;
int pos_lb = 0;
int pos_rf = 0;
int pos_rb = 0;


void setup() {
 Serial1.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 Serial2.begin(115200);      // Roboteq SDC2130 COM (Must be 115200) 

 pinMode(lf,INPUT);
 pinMode(lb,INPUT);
 pinMode(rf,INPUT);
 pinMode(rb,INPUT);
 attachInterrupt(digitalPinToInterrupt(lf),readEncoder_lf,RISING);
 attachInterrupt(digitalPinToInterrupt(lb),readEncoder_lb,RISING);
 attachInterrupt(digitalPinToInterrupt(rf),readEncoder_rf,RISING);
 attachInterrupt(digitalPinToInterrupt(rb),readEncoder_rb,RISING);
 nh.initNode();
 nh.advertise(encoder);
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}


void loop() { 

  Motor_Controller drive;
  //drive.init(nh);
  drive.movement();


  //publishing encoder value to /encoder
  int_msg.data = pos_lf;
  encoder.publish( &int_msg); 
  nh.spinOnce();
  delay(100);
}








//every encoder inside motor gets function
void readEncoder_lf(){
  int b = digitalRead(4);
  if(b > 0){
    pos_lf++;
  }
  else{
    pos_lf--;
  }
}
void readEncoder_lb(){
  int b = digitalRead(5);
  if(b > 0){
    pos_lb++;
  }
  else{
    pos_lb--;
  }
}
void readEncoder_rf(){
  int b = digitalRead(17);
  if(b > 0){
    pos_rf++;
  }
  else{
    pos_rf--;
  }
}
void readEncoder_rb(){
  int b = digitalRead(20);
  if(b > 0){
    pos_rb++;
  }
  else{
    pos_rb--;
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
  Serial1.print("!G");
  Serial1.print(" ");
  Serial1.print(chanel);
  Serial1.print(" ");
  Serial1.println(velocity);
}

void motor_controller_back (int chanel, int velocity){
  Serial2.print("!G");
  Serial2.print(" ");
  Serial2.print(chanel);
  Serial2.print(" ");
  Serial2.println(velocity);
}
