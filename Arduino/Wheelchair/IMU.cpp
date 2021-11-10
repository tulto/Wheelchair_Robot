#include "IMU.h"


// setting topics for each publisher of IMU Sensor
IMU::IMU() 
: imu_orient("/imu/orient", &orient_msg),
imu_accel("/imu/accel", &accel_msg),
imu_gyro("/imu/gyro", &gyro_msg),
imu_head("/imu/head", &head_msg),
cali("/imu/cali", &cali_msg)
{}


void IMU::init(ros::NodeHandle& nh){

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.logwarn("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  
  //looking for Calibration Data inside der EEPROM  
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
  //setting up all adverticers to publish in ROS
  nh.advertise(imu_orient);
  nh.advertise(imu_accel);
  nh.advertise(imu_gyro);
  nh.advertise(imu_head);
  nh.advertise(cali);
}

//call up IMU Data and publish it 
//Publishing Accelerometer, Gyro and Quaternion
void IMU::publish_imu_data(ros::NodeHandle& nh){
 
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - °/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  
  //splitting of the IMU data for a faster transfer of the data into ROS 
  // send Orientation data
  orient_msg.x = quat.x();
  orient_msg.y = quat.y();
  orient_msg.z = quat.z();
  orient_msg.w = quat.w();
  imu_orient.publish( &orient_msg ); // send orientation

  // send acceleration data
  accel_msg.x = accel.x();
  accel_msg.y = accel.y();
  accel_msg.z = accel.z();
  imu_accel.publish( &accel_msg ); // send acceleration

  // send gyroscope data
  gyro_msg.x = gyro.x()/57.296;
  gyro_msg.y = gyro.y()/57.296;
  gyro_msg.z = gyro.z()/57.296;
  imu_gyro.publish( &gyro_msg ); // send gyro

  // sending header to get timing right
  head_msg.seq = seq;
  head_msg.stamp = nh.now();
  head_msg.frame_id = "base_link";
  imu_head.publish( &head_msg );
  seq++;
}

//publish if IMU is fully Calibrated (best Case is 3 3 3)
void IMU::publish_imu_cali(){
  uint8_t system, cal_gyro, cal_accel, cal_mag = 0;
  bno.getCalibration(&system, &cal_gyro, &cal_accel, &cal_mag);
  cali_msg.x = cal_gyro;
  cali_msg.y = cal_accel;
  cali_msg.z = cal_mag;
  cali.publish( &cali_msg );
}
