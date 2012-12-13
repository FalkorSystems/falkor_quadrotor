#include "ros/ros.h"
#include "falkor_msgs/Pwm.h"

falkor_msgs::Pwm::ConstPtr pwm_in;
falkor_msgs::Pwm::ConstPtr pwm_cmd;

void pwm_in_cb(falkor_msgs::Pwm::ConstPtr in_msg)
{
  pwm_in = in_msg;
}

void pwm_cmd_cb( const falkor_msgs::Pwm::ConstPtr in_msg)
{
  pwm_cmd = in_msg;
}

int main(int argc, char **argv)
{
  ros::init( argc, argv, "pwm_switch" );
  ros::NodeHandle n;

  falkor_msgs::Pwm pwm_default;
  for( int i = 0; i < 7; i++ )
    {
      pwm_default.pwm.push_back( 1500 );
    }

  pwm_in = falkor_msgs::Pwm::ConstPtr( new falkor_msgs::Pwm( pwm_default ) );
  pwm_cmd = falkor_msgs::Pwm::ConstPtr( new falkor_msgs::Pwm( pwm_default ) );

  ros::Publisher pwm_out_pub = n.advertise<falkor_msgs::Pwm>("pwm_out", 1000);
  ros::Subscriber pwm_in_sub = n.subscribe( "pwm_in", 1000, pwm_in_cb );
  ros::Subscriber pwm_cmd_sub = n.subscribe( "pwm_cmd", 1000, pwm_cmd_cb );
  int chan_5_fix = 1870;

  ros::Rate loop_rate(50);

  while (ros::ok())
    {
      falkor_msgs::Pwm::Ptr pwm_out(new falkor_msgs::Pwm);

      if(pwm_in->pwm.size() >= 5)
	{
	  if (pwm_in->pwm[4] > 1500)
	    {
	      pwm_out->pwm = pwm_in->pwm;
	      pwm_out->pwm[4] = chan_5_fix;
	    }
	  else
	    {
	      pwm_out->pwm = pwm_cmd->pwm;
	      pwm_out->pwm[4] = chan_5_fix;
	    }
	  pwm_out_pub.publish( pwm_out );
	}

      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
