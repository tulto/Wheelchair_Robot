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

Echo_Sensor sensor; //definiere Echo_Sensor Class

Motor_Controller::Motor_Controller() 
: subscriber_motion("/cmd_vel", &Motor_Controller::callback_motion, this)
{
  nh.subscribe(subscriber_motion);
}

// set depending on subscribed msg /cmd_vel values of array motion
void Motor_Controller::callback_motion(const geometry_msgs::Twist& msg) {
  x_ = msg.linear.x;
  y_ = msg.linear.y;
  t_ = msg.angular.z;
}

//kann vielleicht entfernt werden
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

//die über ros geschickten Werte werden für die Bewegung übernommen
void Motor_Controller::set_sent_movement(){
  motion[0] = x_;
  motion[1] = y_;
  motion[2] = t_;
}


//manuelles setzen der Bewegungen
void Motor_Controller::set_movement(float x, float y, float turning){
  motion[0] = x;
  motion[1] = y;
  motion[2] = turning;
}

//ausgeben der momentan gegebenen Werte der Bewegung
float* Motor_Controller::get_movement(){  
  return motion;
}

//Bewegungswerte x,y,t werden in Bewegung umgewandelt, sowie abgefragt ob Echosensoren eine Mauer erkennen 
void Motor_Controller::movement(){

  //gegebenen motion Werte werden abgefragt um zu sehen ob der Echo Sensor eine Mauer erkennt und x,y oder t auf null gesetzt werden müssen
  float x = sensor.blocking_path(motion[0], motion[1], motion[2])[0] * max_speed;
  float y = sensor.blocking_path(motion[0], motion[1], motion[2])[1] * max_speed;
  float turning = sensor.blocking_path(motion[0], motion[1], motion[2])[2] * max_speed;  

  Motor_Controller::control_front(1,(x - turning) + y);
  Motor_Controller::control_front(2,(x + turning) - y);
  Motor_Controller::control_back(1, (x - turning) - y);
  Motor_Controller::control_back(2, (x + turning) + y);
  delay(100);
}
