#ifndef FALKOR_DRIVERS_PWM_INPUT_H
#define FALKOR_DRIVERS_PWM_INPUT_H

#include "ros/ros.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/noncopyable.hpp>
#include <string>
#include <iostream>
#include "falkor_msgs/Pwm.h"

class PwmInput
{
private:
  boost::shared_ptr<boost::asio::serial_port> serialPtr_;
  boost::asio::streambuf buffer_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pwm_in_pub_;
  ros::AsyncSpinner spinner_;

public:
  PwmInput(boost::shared_ptr<boost::asio::serial_port> serialPtr,
	   ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~PwmInput(void);
  void processLine(const std::string &line);
  void readHandler( const boost::system::error_code &error,
		    size_t bytes_transferred );
};

#endif /* FALKOR_DRIVERS_PWM_INPUT_H */
