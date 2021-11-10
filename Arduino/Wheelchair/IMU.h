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
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>


class IMU {
  private:
  uint32_t seq = 0;
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
  //                                   id, address
  Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


  // Publisher für IMU daten
  
  // splitting of the IMU data for a faster transfer of the data into ROS 
  // all msgs are defined
  geometry_msgs::Quaternion orient_msg;
  geometry_msgs::Vector3 accel_msg;
  geometry_msgs::Vector3 gyro_msg;
  std_msgs::Header head_msg;

  // publisher are defined
  ros::Publisher imu_orient;
  ros::Publisher imu_accel;
  ros::Publisher imu_gyro;
  ros::Publisher imu_head;

  //Publisher für Kallibrierungsdaten
  geometry_msgs::Vector3 cali_msg;
  ros::Publisher cali;


  public:
  IMU();
  void init(ros::NodeHandle& nh);

  void publish_imu_data(ros::NodeHandle& nh);
  void publish_imu_cali();
};

#endif
