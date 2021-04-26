  // Command Structure
  //  !G [nn] mm
  //  | - "!G" is the "Go" commnand
  //      | - nn = Motor Channel
  //          | - mm = Motor Power (-1000 to 1000)
  //  NOTE: Each command is completed by a carriage
  //        return. CR = dec 12, hex 0x0D
 
  // Ramp Forward
  //exapel

#include "Motor_Controller.h"

Motor_Controller::Motor_Controller() 
: subscriber_("/cmd_vel", &Motor_Controller::callback_stearing, this)
{}

// set depending on subscribed msg /cmd_vel values of array motion
void Motor_Controller::callback_stearing(const geometry_msgs::Twist& msg) {
    motion[0] = msg.linear.x;  
    motion[1] = msg.angular.z;
    motion[2] = msg.linear.y;
    motion[3] = msg.linear.z;
  }
void Motor_Controller::init(ros::NodeHandle& nh)
{
  nh.subscribe(subscriber_);
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

void Motor_Controller::movement(){
  float x = motion[0] * max_speed;
  float turning = motion[1] * max_speed;
  float y = motion[2] * max_speed;
  float z = motion[3] * max_speed;
  Motor_Controller::control_front(1,(x + turning)   );
  Motor_Controller::control_front(2,(-x + turning)  );
  Motor_Controller::control_back(1,(x + turning)    );
  Motor_Controller::control_back(2,(-x + turning)   );
  delay(100);
}
