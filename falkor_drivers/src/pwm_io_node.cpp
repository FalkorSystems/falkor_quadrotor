#include "falkor_drivers/pwm_input.h"
#include "falkor_drivers/pwm_pololu.h"

int main(int argc, char**argv)
{
  ros::init(argc, argv, "PwmInput");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  boost::asio::io_service io;
  boost::shared_ptr<boost::asio::serial_port> serialPtr(new boost::asio::serial_port( io, "/dev/ttyO1" ) );
  serialPtr->set_option(boost::asio::serial_port_base::baud_rate(115200));

  PwmInput pwmInput( serialPtr, nh, nh_private );
  PwmPololu pwmPololu( serialPtr, nh, nh_private );

  while( ros::ok() )
    io.run_one();
}
