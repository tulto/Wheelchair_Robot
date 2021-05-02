#include "Echo_Sensor.h"


Echo_Sensor::Echo_Sensor() 
: subscriber_echo_("/echo_sensor", &Echo_Sensor::callback_sensor, this)
{
  nh.subscribe(subscriber_echo_);
}

void Echo_Sensor::callback_sensor(const services_and_messages::Echosensors& msg) {
  //will get boolenarray to defined boolenarray "sensors"
  for (int i=0; i<4; i++){
    sensors[i] = msg.echo_dir[i];
  }
}

//ausgabe der Sensorwerte als bool array [0] = front, [1] = links, [2] = rechts, [3] = hinten 
bool* Echo_Sensor::get_sensor(){
  return sensors;
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
  
  if (sensors[0] == true & x>0){ //front blockiert 
    x=0;
  }else if (sensors[1] == true & y>0){ //links blockiert
    y=0;
  }else if (sensors[2] == true & y<0){ //rechts blockiert
    y=0;
  }else if (sensors[3] == true & x<0){ //hinten blockiert
    x=0;
  }

  float motion[3] = {x,y,t};
  return motion;
}
