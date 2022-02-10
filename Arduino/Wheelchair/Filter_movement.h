#ifndef Filter_movement_H
#define Filter_movement_H


#include <ros.h>

class Filter_movement {
  private:
  bool sensors[4]; //0 front, 1 left, 2 right, 3 back
  bool front_, left_, right_, back_;

  public:
  Filter_movement();
  bool* get_sensor();
  void set_sent_sensor();
  void set_sensor(bool front, bool left, bool right, bool back);
  float* blocking_path(float x, float y, float t);  
};

#endif
