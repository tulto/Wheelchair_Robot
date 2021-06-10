#ifndef Echo_Sensor_H
#define Echo_Sensor_H


#include <ros.h>
#include <services_and_messages/Echosensors.h>

class Echo_Sensor {
  private:
  bool sensors[4]; //0 front, 1 left, 2 right, 3 back
  bool front_, left_, right_, back_;
  ros::Subscriber<services_and_messages::Echosensors, Echo_Sensor> subscriber_echo_;


  public:
  Echo_Sensor();
  void init(ros::NodeHandle& nh);
  void callback_sensor(const services_and_messages::Echosensors& msg);
  bool* get_sensor();
  void set_sent_sensor();
  void set_sensor(bool front, bool left, bool right, bool back);
  float* blocking_path(float x, float y, float t);  
};

#endif
