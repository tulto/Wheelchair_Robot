#include "Echo_Sensor.h"


Echo_Sensor::Echo_Sensor() 
: subscriber_echo_("/echo_sensor", &Echo_Sensor::callback_sensor, this)
{}

void Echo_Sensor::init(ros::NodeHandle& nh){
  //nh.subscribe(subscriber_echo_);
}

void Echo_Sensor::callback_sensor(const services_and_messages::Echosensors& msg_echo) {
  //will get boolenarray to defined boolenarray "sensors"
  sensors[0] = msg_echo.echo_dir[0];
  sensors[1] = msg_echo.echo_dir[1];
  sensors[2] = msg_echo.echo_dir[2];
  sensors[3] = msg_echo.echo_dir[3];
}

//ausgabe der Sensorwerte als bool array [0] = front, [1] = links, [2] = rechts, [3] = hinten 
bool* Echo_Sensor::get_sensor(){
  static bool* x;
  x = sensors;
  return x;
}

//möglichkeit die Sensorwerte künstlich auf true zu setzen
void Echo_Sensor::set_sensor(bool front, bool left, bool right, bool back){
  sensors[0] = front;
  sensors[1] = left;
  sensors[2] = right;
  sensors[3] = back;
}

//abfragen ob sensoren blockiert sind und dementsprechend werte zurück setzen (drehen wird nie blockiert)
//x,y und t Werte muessen seperat zusätzlich noch eingegeben werden
float* Echo_Sensor::blocking_path(float x, float y, float t){

  if (sensors[0] == true && x>0){ //front blockiert 
    x=0;
  }else if (sensors[1] == true && y>0){ //links blockiert
    y=0;
  }else if (sensors[2] == true && y<0){ //rechts blockiert
    y=0;
  }else if (sensors[3] == true && x<0){ //hinten blockiert
    x=0;
  }

  static float motion_filter[3];
  motion_filter[0] = x;
  motion_filter[1] = y;
  motion_filter[2] = t;
 
  return motion_filter; 
}
