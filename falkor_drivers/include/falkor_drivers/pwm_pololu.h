#ifndef FALKOR_DRIVERS_PWM_POLOLU_H
#define FALKOR_DRIVERS_PWM_POLOLU_H

#include "ros/ros.h"
#include <boost/asio.hpp>
#include "falkor_msgs/Pwm.h"

class PwmPololu
{
private:
  boost::shared_ptr<boost::asio::serial_port> serialPtr_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber pwm_out_sub_;
  ros::AsyncSpinner spinner_;

public:
  PwmPololu(boost::shared_ptr<boost::asio::serial_port> serialPtr,
	    ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~PwmPololu(void);
  void setPosition( uint8_t servo_id, int microseconds );
  void pwmOutCallback( falkor_msgs::Pwm::ConstPtr out_msg);
};

#endif /* FALKOR_DRIVERS_PWM_POLOLU_H */

