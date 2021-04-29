  // Command Structure
  //  !G [nn] mm
  //  | - "!G" is the "Go" commnand
  //      | - nn = Motor Channel
  //          | - mm = Motor Power (-1000 to 1000) (reasonable values are +/-250)
  //  NOTE: Each command is completed by a carriage
  //        return. CR = dec 12, hex 0x0D
 
  // Ramp Forward
  //exapel

#include "Motor_Controller.h"
#include "Echo_Sensor.h"

Echo_Sensor sensor;

Motor_Controller::Motor_Controller() 
: subscriber_motion("/cmd_vel", &Motor_Controller::callback_stearing, this)
{
  nh.subscribe(subscriber_motion);
}

// set depending on subscribed msg /cmd_vel values of array motion
void Motor_Controller::callback_stearing(const geometry_msgs::Twist& msg) {
  motion[0] = msg.linear.x;  
  motion[1] = msg.angular.z;
  motion[2] = msg.linear.y;
}

void Motor_Controller::init(ros::NodeHandle& nh)
{
  nh.subscribe(subscriber_motion);
}

// controller in front gets commands chanel(1 = left, 2 = right)
void Motor_Controller::control_front (int chanel, int velocity){
  Serial1.print("!G");
  Serial1.print(" ");
  Serial1.print(chanel);
  Serial1.print(" ");
  Serial1.println(velocity);
}

// controller in back gets commands chanel(1 = left, 2 = right)
void Motor_Controller::control_back (int chanel, int velocity){
  Serial2.print("!G");
  Serial2.print(" ");
  Serial2.print(chanel);
  Serial2.print(" ");
  Serial2.println(velocity);
}

void Motor_Controller::set_movement(float x, float y, float turning){
  x = motion[0] * max_speed;
  y = motion[2] * max_speed;
  turning = motion[1] * max_speed;
}

void Motor_Controller::get_movement(){  
  return motion;
}

void Motor_Controller::movement(){
  float x = motion[0] * max_speed;
  float turning = motion[1] * max_speed;
  float y = motion[2] * max_speed;

  //ausgelesene Sensorwerte werden abgefragt und bearbeitet
  bool *values = sensor.get_values();
  //abfragen ob sensoren blockiert sind und dementsprechend werte zurück setzen 
  //(drehen wird nie blockiert)
  if (values[0] == true & x>0){ //front blockiert 
    x=0;
  }else if (values[1] == true & y>0){ //links blockiert
    y=0;
  }else if (values[2] == true & y<0){ //rechts blockiert
    y=0;
  }else if (values[3] == true & x<0){ //hinten blockiert
    x=0;
  }

  Motor_Controller::control_front(1,(x - turning) + y);
  Motor_Controller::control_front(2,(x + turning) - y);
  Motor_Controller::control_back(1, (x - turning) - y);
  Motor_Controller::control_back(2, (x + turning) + y);
  delay(100);
}
