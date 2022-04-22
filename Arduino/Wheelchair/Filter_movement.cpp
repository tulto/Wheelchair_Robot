#include "Filter_movement.h"


Filter_movement::Filter_movement() 
{}

//ausgabe der Sensorwerte als bool array [0] = front, [1] = links, [2] = rechts, [3] = hinten 
bool* Filter_movement::get_sensor(){
  static bool* x;
  x = sensors;
  return x;
}

//möglichkeit die Sensorwerte künstlich auf true zu setzen
void Filter_movement::set_sensor(bool front, bool left, bool right, bool back){
  sensors[0] = front;
  sensors[1] = left;
  sensors[2] = right;
  sensors[3] = back;
}

//abfragen ob sensoren blockiert sind und dementsprechend werte zurück setzen (drehen wird nie blockiert)
//x,y und t Werte muessen seperat zusätzlich noch eingegeben werden
float* Filter_movement::blocking_path(float x, float y, float t){

  if (sensors[0] == true && x>0){ //front blockiert 
    x=0;
  }
  if (sensors[1] == true && y>0){ //links blockiert
    y=0;
  }
  if (sensors[2] == true && y<0){ //rechts blockiert
    y=0;
  }
  if (sensors[3] == true && x<0){ //hinten blockiert
    x=0;
  }

  static float motion_filter[3];
  motion_filter[0] = x;
  motion_filter[1] = y;
  motion_filter[2] = t;
 
  return motion_filter; 
}
