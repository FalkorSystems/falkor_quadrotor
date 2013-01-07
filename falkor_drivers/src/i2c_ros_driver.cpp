#include <falkor_drivers/i2c_ros_driver.h>

I2CRosDriver::I2CRosDriver(ros::NodeHandle nh_,
			   ros::NodeHandle nhPrivate_,
			   int i2cbus_ ) :
  nh( nh_ ),
  nhPrivate( nhPrivate_ ),
  i2c(i2cbus_),
  accelerometer( 0x53, &i2c ),
  gyrometer( 0x68, &i2c ),
  barometer( 0x77, &i2c ),
  magnetometer( 0x1E, &i2c )
{
  int i2cBus;
  if(!nhPrivate.getParam( "i2cbus", i2cBus))
    i2cBus = 3;

  // Calibrate magnetometer
  double center_x, center_y;
  double axes_x, axes_y;
  double angle;

  if(!nhPrivate.getParam( "mag_calib/center/x", center_x ))
    center_x = 0.0;

  if(!nhPrivate.getParam( "mag_calib/center/y", center_x ))
    center_y = 0.0;

  if(!nhPrivate.getParam( "mag_calib/axes/x", axes_x ))
    axes_x = 1.0;

  if(!nhPrivate.getParam( "mag_calib/axes/y", axes_y ))
    axes_y = 1.0;

  if(!nhPrivate.getParam( "mag_calib/angle", angle ))
    angle = 0.0;

  int stp_x, stp_y, stp_z;
  if(!nhPrivate.getParam( "mag_calib/stp/x", stp_x ))
    stp_x = 466;

  if(!nhPrivate.getParam( "mag_calib/stp/x", stp_y ))
    stp_y = 433;

  if(!nhPrivate.getParam( "mag_calib/stp/x", stp_z ))
    stp_z = 453;

  magnetometer.setCalibration( center_x, center_y, axes_x, axes_y, angle, stp_x, stp_y, stp_z );

  if(!nhPrivate.getParam( "mag_topic", magTopic ))
    magTopic = "magnetic";

  if(!nhPrivate.getParam( "imu_topic", imuTopic ))
    imuTopic = "raw_imu";

  if(!nhPrivate.getParam( "baro_topic", baroTopic ))
    baroTopic = "altimeter";

  if(!nhPrivate.getParam( "tf_prefix", tfPrefix ))
    tfPrefix = "";

  if(!nhPrivate.getParam( "mag_frame", magFrame ))
    magFrame = "magnetometer";

  if(!nhPrivate.getParam( "imu_frame", imuFrame ))
    imuFrame = "imu";

  if(!nhPrivate.getParam( "baro_frame", baroFrame ))
    baroFrame = "altimeter";

  imuPub = nh.advertise<sensor_msgs::Imu>(imuTopic, 1);
  magPub = nh.advertise<geometry_msgs::Vector3Stamped>(magTopic, 1);
  baroPub = nh.advertise<falkor_msgs::Altimeter>(baroTopic, 1);
}

void I2CRosDriver::start(void) {
  accelerometer.init();
  gyrometer.init();
  barometer.init();
  magnetometer.init();

  imuTimer = nh.createTimer( ros::Duration(1.0/125), &I2CRosDriver::imuCallback, this );
  magTimer = nh.createTimer( ros::Duration(1.0/15), &I2CRosDriver::magCallback, this );
  baroTimer = nh.createTimer( ros::Duration(1.0/25), &I2CRosDriver::baroCallback, this );
}

void I2CRosDriver::imuCallback(const ros::TimerEvent &event) {
  std::vector<double> accelData = accelerometer.read();
  std::vector<double> gyroData = gyrometer.read();

  sensor_msgs::Imu msg;
  msg.header.stamp = event.current_real;
  msg.header.frame_id = tfPrefix + "/" + imuFrame;
  msg.orientation.x = msg.orientation.y = msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.angular_velocity.x = gyroData[0];
  msg.angular_velocity.y = -gyroData[1];
  msg.angular_velocity.z = -gyroData[2];
  msg.linear_acceleration.x = accelData[0];
  msg.linear_acceleration.y = accelData[1];
  msg.linear_acceleration.z = accelData[2];

  imuPub.publish( msg );
}
      
void I2CRosDriver::magCallback(const ros::TimerEvent &event) {
  std::vector<double> magData = magnetometer.read();

  geometry_msgs::Vector3Stamped msg;
  msg.header.stamp = event.current_real;
  msg.header.frame_id = tfPrefix + "/" + magFrame;
  msg.vector.x = magData[0];
  msg.vector.y = magData[1];
  msg.vector.z = magData[2];

  magPub.publish( msg );
}

void I2CRosDriver::baroCallback(const ros::TimerEvent &event) {
  std::vector<double> baroData = barometer.read();

  falkor_msgs::Altimeter msg;
  msg.header.stamp = event.current_real;
  msg.header.frame_id = tfPrefix + "/" + baroFrame;
  msg.altitude = baroData[2];
  msg.pressure = baroData[1];
  
  baroPub.publish( msg );
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "i2c_ros_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  I2CRosDriver i2CRosDriver(nh, nh_private, 3);
  i2CRosDriver.start();

  ros::spin();
};

