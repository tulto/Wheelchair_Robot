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

bool* Echo_Sensor::get_values(){
  return sensors;
}

void Echo_Sensor::set_values(bool front, bool left, bool right, bool back){
  sensors[0] = front;
  sensors[1] = left;
  sensors[2] = right;
  sensors[3] = back;
}

void Echo_Sensor::blocking_path(){
  if (sensors[0] == true){
    
  }
}
