#include "ros/ros.h"
#include "falkor_msgs/Pwm.h"

falkor_msgs::Pwm pwm_in;
falkor_msgs::Pwm pwm_cmd;

void pwm_in_cb(const falkor_msgs::Pwm &in_msg)
{
  pwm_in = in_msg;
}

void pwm_cmd_cb( const falkor_msgs::Pwm &in_msg)
{
  pwm_cmd = in_msg;
}

int main(int argc, char **argv)
{
  ros::init( argc, argv, "pwm_switch" );
  ros::NodeHandle n;

  for( int i = 0; i < 7; i++ )
    {
      pwm_in.pwm.push_back( 1500 );
      pwm_cmd.pwm.push_back( 1500 );
    }

  ros::Publisher pwm_out_pub = n.advertise<falkor_msgs::Pwm>("pwm_out", 1000);
  ros::Subscriber pwm_in_sub = n.subscribe( "pwm_in", 1000, pwm_in_cb );
  ros::Subscriber pwm_cmd_sub = n.subscribe( "pwm_cmd", 1000, pwm_cmd_cb );
  int chan_5_fix = 1870;

  ros::Rate loop_rate(50);

  while (ros::ok())
    {
      falkor_msgs::Pwm pwm_out;

      if(pwm_in.pwm.size() >= 5)
	{
	  if (pwm_in.pwm[4] > 1500)
	    {
	      pwm_out = pwm_in;
	      pwm_out.pwm[4] = chan_5_fix;
	    }
	  else
	    {
	      pwm_out = pwm_cmd;
	      pwm_out.pwm[4] = chan_5_fix;
	    }
	  pwm_out_pub.publish( pwm_out );
	}

      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
