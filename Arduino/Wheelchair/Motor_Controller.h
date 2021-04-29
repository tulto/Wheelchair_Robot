#ifndef Motorcontroll_H
#define Motorcontroll_H

//#include "arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

class Motor_Controller {
  private:
  int max_speed = 50;
  float motion[3];
  ros::Subscriber<geometry_msgs::Twist, Motor_Controller> subscriber_motion; 
  ros::NodeHandle& nh;

  public:
  Motor_Controller();
  void callback_stearing(const geometry_msgs::Twist& msg);   
  void init(ros::NodeHandle& nh);
  void control_front(int chanel, int velocity);
  void control_back(int chanel, int velocity);
  void set_movement(float x, float y, float turning);
  void get_movement();
  void movement();
  
};

#endif
