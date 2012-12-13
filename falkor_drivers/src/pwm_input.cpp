#include "ros/ros.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string>
#include <iostream>
#include "falkor_msgs/Pwm.h"

class PwmInput
{
private:
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  boost::asio::streambuf buffer_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pwm_in_pub_;
  ros::AsyncSpinner spinner_;

public:
  PwmInput(std::string port, unsigned int baud_rate,
	   ros::NodeHandle nh, ros::NodeHandle nh_private)
    : io_(), serial_(io_, port), nh_(nh), nh_private_(nh_private), spinner_(1)
  {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    pwm_in_pub_ = nh_.advertise<falkor_msgs::Pwm>("pwm_in", 1);

    boost::asio::async_read_until( serial_, buffer_,
				   "\r\n",
				   boost::bind( &PwmInput::readHandler,
						this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));


  }

  void processLine(const std::string &line)
  {
    if(! line.size() )
      return;

    std::vector<std::string> strs;
    boost::split(strs,line,boost::is_any_of(","));
    falkor_msgs::Pwm::Ptr pwm_in_msg(new falkor_msgs::Pwm);

    for(std::vector<std::string>::const_iterator iter = strs.begin();
	iter < strs.end(); iter++ )
      {
	int pwm_value = atoi(iter->c_str());
	pwm_in_msg->pwm.push_back( pwm_value );
      }
    pwm_in_pub_.publish( pwm_in_msg );
  }
    
  void readHandler( const boost::system::error_code &error,
		    size_t bytes_transferred )
  {
    std::istream is(&buffer_);
    std::string input;
    is >> input;


    processLine( input );
    boost::asio::async_read_until( serial_, buffer_,
				   "\r\n",
				   boost::bind( &PwmInput::readHandler,
						this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
  }

  void run(void)
  {
    spinner_.start();
    while( ros::ok() )
      io_.run_one();
  }
};

int main(int argc, char**argv)
{
  ros::init(argc, argv, "PwmInput");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  PwmInput pwmInput( "/dev/ttyUSB0", 57600, nh, nh_private );
  pwmInput.run();
}
