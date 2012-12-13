#include "falkor_drivers/pwm_pololu.h"

PwmPololu::PwmPololu(boost::shared_ptr<boost::asio::serial_port> serialPtr,
		     ros::NodeHandle nh, ros::NodeHandle nh_private):
  serialPtr_(serialPtr), nh_(nh), nh_private_(nh_private), spinner_(1)
{
  pwm_out_sub_ = nh_.subscribe( "pwm_out", 1, &PwmPololu::pwmOutCallback, this );

  // send detect baud rate command
  uint8_t msg[1] = { 0xAA };
  boost::asio::write( *serialPtr_,
		      boost::asio::buffer(msg, 1) );

  spinner_.start();
}

PwmPololu::~PwmPololu(void)
{
  spinner_.stop();
}

void PwmPololu::setPosition( uint8_t servo_id, int microseconds )
{
  int position_value = microseconds * 4;
  uint8_t data2 = ( position_value >> 7 ) & 0x7F;
  uint8_t data1 = position_value & 0x7F;
  uint8_t command_byte = 0x84;
  uint8_t msg[4] = { command_byte, servo_id, data1, data2 };
  ROS_DEBUG_STREAM( "Sending " << microseconds << " to " << (int) servo_id );

  boost::asio::write( *serialPtr_,
		      boost::asio::buffer( msg, 4 ) );
}

void PwmPololu::pwmOutCallback( falkor_msgs::Pwm::ConstPtr out_msg )
{
  for(uint8_t servo_id = 0; servo_id < out_msg->pwm.size(); servo_id++)
    {
      setPosition(servo_id, out_msg->pwm[servo_id]);
    }
}

