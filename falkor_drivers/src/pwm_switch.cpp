#include "falkor_drivers/pwm_switch.h"

PwmSwitch::PwmSwitch(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  spinner_(1)
{
  falkor_msgs::Pwm pwm_default;
  for( int i = 0; i < 7; i++ )
    {
      pwm_default.pwm.push_back( 1500 );
    }

  pwm_in_ = falkor_msgs::Pwm::ConstPtr( new falkor_msgs::Pwm( pwm_default ) );
  pwm_cmd_ = falkor_msgs::Pwm::ConstPtr( new falkor_msgs::Pwm( pwm_default ) );

  pwm_out_pub_ = nh_.advertise<falkor_msgs::Pwm>("pwm_out", 1);
  pwm_in_sub_ = nh_.subscribe( "pwm_in", 1, &PwmSwitch::pwmInCallback, this );
  pwm_cmd_sub_ = nh_.subscribe( "pwm_cmd", 1, &PwmSwitch::pwmCmdCallback, this );
  chan_5_fix_ = 1870;
  timer_ = nh_.createTimer( ros::Duration(0.020), &PwmSwitch::loopOnce, this );

  spinner_.start();
}

void PwmSwitch::pwmInCallback(falkor_msgs::Pwm::ConstPtr in_msg)
{
  pwm_in_ = in_msg;
}

void PwmSwitch::pwmCmdCallback( const falkor_msgs::Pwm::ConstPtr in_msg)
{
  pwm_cmd_ = in_msg;
}

void PwmSwitch::loopOnce(const ros::TimerEvent &event)
{
  falkor_msgs::Pwm::Ptr pwm_out(new falkor_msgs::Pwm);

  if(pwm_in_->pwm.size() >= 5)
    {
      if (pwm_in_->pwm[4] > 1500)
	{
	  pwm_out->pwm = pwm_in_->pwm;
	  pwm_out->pwm[4] = chan_5_fix_;
	}
      else
	{
	  pwm_out->pwm = pwm_cmd_->pwm;
	  pwm_out->pwm[4] = chan_5_fix_;
	}
      pwm_out_pub_.publish( pwm_out );
    }
}
