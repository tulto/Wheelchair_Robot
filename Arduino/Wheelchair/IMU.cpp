#include "IMU.h"



IMU::IMU() 
: imu("/imu/data", &imu_msg),
cali("/imu/cali", &cali_msg)
{}


void IMU::init(ros::NodeHandle& nh){

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.logwarn("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    nh.loginfo("No Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
  else
  {
    //Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    nh.loginfo("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    nh.loginfo("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }

  nh.initNode();
  nh.advertise(imu);
  nh.advertise(cali);
}

void IMU::publish_imu_data(ros::NodeHandle& nh){
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion quat = bno.getQuat();

  // Header
  imu_msg.header.seq = seq;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "base_link";

  // Quaternion data
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();
  for (int i=0; i<9;i++){
    imu_msg.orientation_covariance[i] = orientarion_covariance[i];
  }

  // Winkelgeschwindigkeit
  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();
  for (int i=0; i<9;i++){
    imu_msg.angular_velocity_covariance[i] = gyro_covariance[i];
  }

  // lineare Beschleunigung
  imu_msg.linear_acceleration.x = linear.x();
  imu_msg.linear_acceleration.y = linear.y();
  imu_msg.linear_acceleration.z = linear.z(); 
  for (int i=0; i<9;i++){
    imu_msg.linear_acceleration_covariance[i] = linear_covariance[i];
  }
  
  imu.publish( &imu_msg );

  seq++;
  
}


void IMU::publish_imu_cali(){
  uint8_t system, cal_gyro, cal_accel, cal_mag = 0;
  bno.getCalibration(&system, &cal_gyro, &cal_accel, &cal_mag);
  cali_msg.x = cal_gyro;
  cali_msg.y = cal_accel;
  cali_msg.z = cal_mag;
  cali.publish( &cali_msg );
}
