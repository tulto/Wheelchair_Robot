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


Motor_Controller::Motor_Controller() 
: subscriber_motion_("/cmd_vel", &Motor_Controller::callback_motion, this),
encoder("/encoder", &encoder_msg)
{}

void Motor_Controller::init(ros::NodeHandle& nh){
  nh.initNode();
  nh.subscribe(subscriber_motion_);
  nh.advertise(encoder);
}

/***************************************************************
Motor Part
***************************************************************/
// set depending on subscribed msg /cmd_vel values of array motion
void Motor_Controller::callback_motion(const geometry_msgs::Twist &msg_motion) {
  x_ = msg_motion.linear.x;
  y_ = msg_motion.linear.y;
  t_ = msg_motion.angular.z;
  timer = millis();
}

//controller in front gets commands chanel(1 = left, 2 = right)
void Motor_Controller::control_front (int chanel, int velocity){
  Serial1.print("!G");
  Serial1.print(" ");
  Serial1.print(chanel);
  Serial1.print(" ");
  Serial1.println(velocity);
}

//controller in back gets commands chanel(1 = left, 2 = right)
void Motor_Controller::control_back (int chanel, int velocity){
  Serial2.print("!G");
  Serial2.print(" ");
  Serial2.print(chanel);
  Serial2.print(" ");
  Serial2.println(velocity);
}

//die über ros geschickten Werte werden für die Bewegung übernommen
void Motor_Controller::set_sent_movement(){
  if (millis() - timer < 300){
    motion[0] = x_;
    motion[1] = y_;
    motion[2] = t_;
  }else{
    motion[0] = 0;
    motion[1] = 0;
    motion[2] = 0;
  }
}

float Motor_Controller::get_x(){
  return motion[0];
}
float Motor_Controller::get_y(){
  return motion[1];
}
float Motor_Controller::get_t(){
  return motion[2];
}


//manuelles setzen der Bewegungen
void Motor_Controller::set_movement(float x, float y, float turning){
  motion[0] = x;
  motion[1] = y;
  motion[2] = turning;
}

//Bewegungswerte x,y,t werden in Bewegung umgewandelt, sowie abgefragt ob Echosensoren eine Mauer erkennen 
void Motor_Controller::movement(){
  //gegebenen motion Werte werden abgefragt um zu sehen ob der Echo Sensor eine Mauer erkennt und x,y oder t auf null gesetzt werden müssen
  float x = motion[0] * 851; //sensor.blocking_path(motion[0], motion[1], motion[2])
  float y = motion[1] * 1000;
  float turning = motion[2] * 756;  

  Motor_Controller::control_front(1, x - turning - y);
  Motor_Controller::control_front(2, x + turning + y);
  Motor_Controller::control_back(1, x - turning + y);
  Motor_Controller::control_back(2, x + turning - y);
  delay(10);

  
  
}

void Motor_Controller::filter_movement(bool front, bool left, bool right, bool back){
  Filter_movement filter;
  filter.set_sensor(front, left, right, back);
  motion[0] = filter.blocking_path(motion[0], motion[1], motion[2])[0];
  motion[1] = filter.blocking_path(motion[0], motion[1], motion[2])[1];
  motion[2] = filter.blocking_path(motion[0], motion[1], motion[2])[2];
}














/*************************************************************************
Encoder Part
*************************************************************************/
//encoder Daten auswerten und als encoder_value[] zurückgeben
void Motor_Controller::send_encoder_count(int timer){
  //definiere temporaeren Speicher
  String content;
  int gleich;
  int doppel;
    
  //auszulesende Chanel werden definiert und abgefragt
  content = "";
  //encoder Werte für Serial1 Schnittstelle
  Serial1.println("?CR "); //?CR [chanel]: relative Encoder Count, ?C [chanel] total Encoder Count
  Serial1.println("!R ");
  //auslesen solange gesenet wird
  Serial1.setTimeout(3);
  content = Serial1.readString();
  gleich = content.indexOf('=');
  doppel = content.indexOf(':');
  encoder_value[0] = content.substring(gleich+1,doppel).toInt();
  encoder_value[1] = content.substring(doppel+1).toInt();
  
  
  //encoder Werte für Serial2 Schnittstelle
  content = "";
  Serial2.println("?CR ");
  Serial2.println("!R ");  
  //auslesen solange gesenet wird
  Serial2.setTimeout(3);
  content = Serial2.readString();
  gleich = content.indexOf('=');
  doppel = content.indexOf(':');
  encoder_value[2] = content.substring(gleich+1,doppel).toInt();
  encoder_value[3] = content.substring(doppel+1).toInt();


  //encoder_values werden in encoder_msg umgewandelt
  encoder_msg.time = timer;
  for (int i = 0; i<4; i++){
    encoder_msg.encoder_wheel[i] = encoder_value[i];
  }
  encoder.publish(&encoder_msg);
/*  
  //nur etwas senden wenn Encoder Werte nicht alle Null sind
  if (encoder_value[0] != 0 && encoder_value[1] != 0 && encoder_value[2] != 0 && encoder_value[3] != 0){
    //encoder_values werden in encoder_msg umgewandelt
    for (int i = 0; i<4; i++){
      encoder_msg.encoder_wheel[i] = encoder_value[i];
    }
    encoder.publish(&encoder_msg);
  }
*/

}

//alle encoder Werte werden ausgelesen und gesendet (erst sinnvoll direkt nach send_encoder_value nutzbar)
int* Motor_Controller::get_encoder_count(){
  return encoder_value;
}
