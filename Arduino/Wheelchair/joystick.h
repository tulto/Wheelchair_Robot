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
    
    float x, y, t;
    bool b;
    
    int threshold;
    

  public: 
    //declaring constructor and destructor
    Joystick(short int x_axis = A0, short int y_axis = A1, short int t_axis = A2, short int button = 7);

    //declaring needed functions
    void init(ros::NodeHandle& nh);
    bool movement();
    float calculate(int analogue, float range);
    float x_velocity();
    float y_velocity();
    float t_velocity();
    void deadzone();
    
  
};


#endif
