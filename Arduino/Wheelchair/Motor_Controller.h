#ifndef Motorcontroll_H
#define Motorcontroll_H

//#include "arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <services_and_messages/Encoder.h>

class Motor_Controller {
  private:
  int max_speed = 50;
  float motion[3]={0};
  int encoder_value[4]={0}; 
  float x_, y_, t_;
  ros::Subscriber<geometry_msgs::Twist, Motor_Controller> subscriber_motion; 
  
  services_and_messages::Encoder encoder_msg;
  ros::Publisher encoder;
  
  ros::NodeHandle& nh;

  public:
  Motor_Controller();

/***************************************************************
Motor Part
***************************************************************/
  void callback_motion(const geometry_msgs::Twist& msg);   
  void init(ros::NodeHandle& nh);
  void set_sent_movement();
  void set_movement(float x, float y, float turning);
  float* get_movement();
  void control_front(int chanel, int velocity);
  void control_back(int chanel, int velocity);
  void movement();

  void planed_position(int position_wheel[4]);


/***************************************************************
Encoder Part
***************************************************************/
  void send_encoder_count();
  int* get_encoder_count();
  
};

#endif
