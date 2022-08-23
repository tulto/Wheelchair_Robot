#ifndef Motorcontroll_H
#define Motorcontroll_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <services_and_messages/Encoder.h>
#include "Filter_movement.h"
#include "Arduino.h"


class Motor_Controller {
  private:
  //Motor
  int max_speed = 800;
  float motion[3];
  unsigned long timer = 0;
  float x_, y_, t_;
  ros::Subscriber<geometry_msgs::Twist, Motor_Controller> subscriber_motion_; 
  
  //encoder  
  services_and_messages::Encoder encoder_msg;
  ros::Publisher encoder;
  int encoder_value[4]={0}; 
 
  ros::NodeHandle nh;
  
  
  public:
   
  Motor_Controller();

/***************************************************************
Motor Part
***************************************************************/
  void callback_motion(const geometry_msgs::Twist &msg);   
  void init(ros::NodeHandle& nh);
  float get_x();
  float get_y();
  float get_t();
  void set_sent_movement();
  void set_movement(float x, float y, float turning);
  void control_front(int chanel, int velocity);
  void control_back(int chanel, int velocity);
  void movement();
  void filter_movement(bool front, bool left, bool right, bool back);



/***************************************************************
Encoder Part
***************************************************************/
  void send_encoder_count(int timer);
  int* get_encoder_count();
  
};

#endif
