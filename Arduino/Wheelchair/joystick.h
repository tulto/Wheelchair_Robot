#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "Arduino.h"
#include <ros.h>

class Joystick
{
  private:
    //declaring private variables
    int x_axis, y_axis, t_axis, button; //save Button states
    short int zero = 0;
    int offset = 512;
    long int timer = 0;
    
    float x, y, t;
    bool b;
    
    int threshold;
    

  public: 
    //declaring constructor and destructor
    Joystick(short int x_axis = A1, short int y_axis = A2, short int t_axis = A3, short int button = 7);

    //declaring needed functions
    void init(ros::NodeHandle& nh);
    bool movement();
    float calculate(int analogue, float range);
    float x_velocity();
    float y_velocity();
    float t_velocity();
    bool pressed_button(int duration = 2000);
    bool get_button();
    void deadzone();
    
  
};


#endif
