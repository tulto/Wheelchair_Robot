#ifndef Motorcontroll_H
#define Motorcontroll_H

//#include "arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

class Motor_Controller {
  private:
  int max_speed = 50;
  float motion[4];
  ros::Subscriber<geometry_msgs::Twist, Motor_Controller> subscriber_;

  public:
  Motor_Controller();
  void callback_stearing(const geometry_msgs::Twist& msg);
  void control_front(int chanel, int velocity);
  void control_back(int chanel, int velocity);
  void movement();
  
};

#endif
