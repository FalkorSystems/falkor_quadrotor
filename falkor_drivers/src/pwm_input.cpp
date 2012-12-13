#include "falkor_drivers/pwm_input.h"

PwmInput::PwmInput(boost::shared_ptr<boost::asio::serial_port> serialPtr,
		   ros::NodeHandle nh, ros::NodeHandle nh_private)
  : serialPtr_(serialPtr), nh_(nh), nh_private_(nh_private), spinner_(1)
{
  pwm_in_pub_ = nh_.advertise<falkor_msgs::Pwm>("pwm_in", 1);

  boost::asio::async_read_until( *serialPtr_, buffer_,
				 "\r\n",
				 boost::bind( &PwmInput::readHandler,
					      this,
					      boost::asio::placeholders::error,
					      boost::asio::placeholders::bytes_transferred));

  spinner_.start();
}

PwmInput::~PwmInput(void)
{
  spinner_.stop();
}

void PwmInput::processLine(const std::string &line)
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
    
void PwmInput::readHandler( const boost::system::error_code &error,
			    size_t bytes_transferred )
{
  std::istream is(&buffer_);
  std::string input;
  is >> input;

  processLine( input );
  boost::asio::async_read_until( *serialPtr_, buffer_,
				 "\r\n",
				 boost::bind( &PwmInput::readHandler,
					      this,
					      boost::asio::placeholders::error,
					      boost::asio::placeholders::bytes_transferred));
}

