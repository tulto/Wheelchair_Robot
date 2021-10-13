#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>


class IMU {
  private:
  uint32_t seq = 0;
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
  //                                   id, address
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

  //Publisher für IMU daten
  sensor_msgs::Imu imu_msg;
  ros::Publisher imu;

  //Publisher für Kallibrierungsdaten
  geometry_msgs::Vector3 cali_msg;
  ros::Publisher cali;

  //Define covarinace parameters
  float orientation_covariance[9];// = {0.001,0,0, 0,0.001,0, 0,0,0.001};
  float gyro_covariance[9];// = {0.0045,0,0, 0,0.0122,0, 0,0,0.0064};
  float linear_covariance[9];// = {0.5,0,0, 0,0.5,0, 0,0,0.5};


  public:
  IMU();
  void init(ros::NodeHandle& nh);

  void publish_imu_data(ros::NodeHandle& nh);
  void publish_imu_cali();
};

#endif
