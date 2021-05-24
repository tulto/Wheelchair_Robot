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
: subscriber_motion("/cmd_vel", &Motor_Controller::callback_motion, this),
encoder("/encoder", &encoder_msg)
{}

void Motor_Controller::init(ros::NodeHandle& nh){
  nh.subscribe(subscriber_motion);
  nh.advertise(encoder);
}

/***************************************************************
Motor Part
***************************************************************/
// set depending on subscribed msg /cmd_vel values of array motion
void Motor_Controller::callback_motion(const geometry_msgs::Twist& msg) {
  x_ = msg.linear.x;
  y_ = msg.linear.y;
  t_ = msg.angular.z;
}

//ausgeben der momentan gegebenen Werte der Bewegung
float* Motor_Controller::get_movement(){  
  return motion;
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

//filter der über Echosensoren bestimmt wird
void Motor_Controller::filter_movement(){
  sensor.set_sensor(true, true, false, false);
  //sensor-set_sent_sensor();
  float *speicher;
  speicher = sensor.blocking_path(motion[0], motion[1], motion[2]);
  for (int i = 0; i<3; i++){  
    motion[i] = speicher[i];
  
  }
}

//Bewegungswerte x,y,t werden in Bewegung umgewandelt, sowie abgefragt ob Echosensoren eine Mauer erkennen 
void Motor_Controller::movement(){
  sensor.set_sensor(true, true, false, false);
  //gegebenen motion Werte werden abgefragt um zu sehen ob der Echo Sensor eine Mauer erkennt und x,y oder t auf null gesetzt werden müssen
  float x = motion[0] * max_speed; //sensor.blocking_path(motion[0], motion[1], motion[2])
  float y = motion[1] * max_speed;
  float turning = motion[2] * max_speed;  

  Motor_Controller::control_front(1,x - turning - y);
  Motor_Controller::control_front(2,x + turning + y);
  Motor_Controller::control_back(1, x - turning + y);
  Motor_Controller::control_back(2, x + turning - y);
  delay(100);
}

//eingeben der geplanten Position (ahnand der schritte des encoders)
void Motor_Controller::planed_position(int position_wheel[4]){
  
}

/***************************************************************
Encoder Part
***************************************************************/
//encoder Daten auswerten und als encoder_value[] zurückgeben
void Motor_Controller::send_encoder_count(){
  //definiere temporaeren Speicher
  String content;
  char character;
  byte modi;
    
    //auszulesende Chanel werden definiert und abgefragt
  content = "";
  Serial1.print("?C "); //?CR [chanel]: relative Encoder Count, ?C [chanel] total Encoder Count

  modi = 0;
  //auslesen solange gesenet wird
  while (Serial1.available() && modi != 4){
    character = Serial1.read();
    if (character == ':'){
      encoder_value[0] = content.toInt();
      content = "";
      modi = 1;
    }
    if (character == '+' || character == '!' || character == '?'){  //Serialread stopen
      encoder_value[1] = content.toInt();
      modi = 4; 
    }
    if (modi == 1){
      content.concat(character);
    }
    if (character == '='){
      modi = 1;
    }        
  }
  
  //encoder Werte für Serial2 Schnittstelle
  content = "";
  Serial2.print("?C ");  
    
  //auslesen solange gesenet wird
  while (Serial2.available() && modi != 4){
    character = Serial2.read();
    if (character == ':'){
      encoder_value[2] = content.toInt();
      content = "";
      modi = 1;
    }
    if (character == '+' || character == '!' || character == '?'){  //Serialread stopen
      encoder_value[3] = content.toInt();
      modi = 4; 
    }
    if (modi == 1){
      content.concat(character);
    }
    if (character == '='){
      modi = 1;
    }
  }
  
  //encoder_values werden in encoder_msg umgewandelt
  for (int i = 0; i<4; i++){
    encoder_msg.encoder_wheel[i] = encoder_value[i];
  }
  encoder.publish(&encoder_msg);
}

//alle encoder Werte werden ausgelesen und gesendet (erst sinnvoll direkt nach send_encoder_value nutzbar)
int* Motor_Controller::get_encoder_count(){
  return encoder_value;
}
