#ifndef Echo_Sensor_H
#define Echo_Sensor_H


#include <ros.h>
#include <services_and_messages/Echosensors.h>

class Echo_Sensor {
  private:
  bool sensors[4];
  ros::Subscriber<services_and_messages::Echosensors, Echo_Sensor> subscriber_echo_;
  ros::NodeHandle& nh;

  public:
  Echo_Sensor();
  void callback_sensor(const services_and_messages::Echosensors& msg);
  bool* get_values();
  void set_values(bool front, bool left, bool right, bool back);
  void blocking_path();
  
};

#endif
