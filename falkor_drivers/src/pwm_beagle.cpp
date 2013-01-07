#include "falkor_drivers/pwm_beagle.h"
#include <errno.h>
#include <stdio.h>

#define PATH_MAX 256

FILE *PwmBeagle::openFile( int i, int j, char *name )
{
  char filename[PATH_MAX];
  FILE *fp;

  snprintf( filename, PATH_MAX, SYSFS_EHRPWM_PREFIX, i, j, "duty_ns" );
  fp = fopen( filename );
  if( fp == NULL )
    throw std::runtime_error( strerror( errno ) );

  return fp;
}

PwmBeagle::openEhrFiles(void)
{
  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 2; j++)
	{
	  ehrDuty_.push_back( openFile( i, j, "duty_ns" ) );
	  ehrPeriod_.push_back( openFile( i, j, "period_ns" ) );
	  ehrPolarity_.push_back( openFile( i, j, "polarity" ) );
	  ehrRun_.push_back( openFile( i, j, "run" ) );
	  ehrRequest_.push_back( openFile( i, j, "request" ) );
	}
    }
}

PwmBeagle::closeAll(const std::vector<FILE *> &files)
{
  for(std::vector<FILE *>::const_iterator iter = files.begin(), iter < files.end(); iter++)
    {
      fclose( *iter );
    }
}

PwmBeagle::closeEhrFiles(void)
{
  closeAll( ehrDuty_ );
  closeAll( ehrPeriod_ );
  closeAll( ehrPolarity_ );
  closeAll( ehrRun_ );
  closeAll( ehrRequest_ );
}

PwmBeagle::set(int value, const std::vector<FILE *> &files )
{
  for(std::vector<FILE *>::const_iterator iter = file.begin(), iter < files.end(); iter++)
    {
      fprintf( *iter, "%d\n", value);
    }
}
PwmBeagle::setFrequency(int freq)
{
  int period = 1000000000 / freq;
  set( period, ehrPeriod_ );
}  

PwmBeagle::setRun(int run)
{
  set( run, ehrRun_ );
}  

PwmBeagle::setRequest(int request)
{
  set( request, ehrRequest_ );
}

PwmBeagle::PwmBeagle(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private), spinner_(1)
{
  pwm_out_sub_ = nh_.subscribe( "pwm_out", 1, &PwmBeagle::pwmOutCallback, this );

  openEhrFiles();

  setRequest(1);
  setFrequency(50);
  setPolarity(0);
  setRun(1);
  spinner_.start();
}

PwmBeagle::~PwmBeagle(void)
{
  spinner_.stop();
  setRun(0);
  setRequest(0);
  closeEhrFiles();
}

void PwmBeagle::setPosition( int servo_id, int microseconds )
{
  int nanoseconds = microseconds * 1000;
  if( servo_id >= ehrDuty_.size() )
    throw std::runtime_error( "servo_id exceeds available PWM ports" );

  fprintf( ehrDuty_[servo_id], "%d\n", nanoseconds );
}

void PwmBeagle::pwmOutCallback( falkor_msgs::Pwm::ConstPtr out_msg )
{
  for(uint8_t servo_id = 0; servo_id < out_msg->pwm.size(); servo_id++)
    {
      setPosition(servo_id, out_msg->pwm[servo_id]);
    }
}

