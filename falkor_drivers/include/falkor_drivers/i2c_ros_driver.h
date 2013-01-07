#ifndef FALKOR_DRIVERS_I2C_ROS_DRIVER_H
#define FALKOR_DRIVERS_I2C_ROS_DRIVER_H

#include "ros/ros.h"
#include <falkor_drivers/sensor.h>
#include <falkor_msgs/Altimeter.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

class I2CRosDriver {
 private:
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate;

  I2CDriver i2c;

  Accelerometer accelerometer;
  Gyrometer gyrometer;
  Barometer barometer;
  Magnetometer magnetometer;

  std::string magTopic;
  std::string imuTopic;
  std::string baroTopic;

  std::string tfPrefix;
  std::string magFrame;
  std::string imuFrame;
  std::string baroFrame;

  ros::Publisher imuPub;
  ros::Publisher magPub;
  ros::Publisher baroPub;

  ros::Timer imuTimer;
  ros::Timer magTimer;
  ros::Timer baroTimer;

 public:
  I2CRosDriver(ros::NodeHandle nh_, ros::NodeHandle nhPrivate_, int i2cbus_);
  void start(void);
  void imuCallback(const ros::TimerEvent &event);
  void magCallback(const ros::TimerEvent &event);
  void baroCallback(const ros::TimerEvent &event);
};

#endif
