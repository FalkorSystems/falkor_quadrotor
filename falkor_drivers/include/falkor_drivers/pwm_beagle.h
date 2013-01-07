#ifndef FALKOR_DRIVERS_PWM_BEAGLE_H
#define FALKOR_DRIVERS_PWM_BEAGLE_H

#include "ros/ros.h"
#include "falkor_msgs/Pwm.h"
#include <stdio.h>
#include <vector>

#define SYSFS_EHRPWM_FILENAME "/sys/class/pwm/ehrpwm.%d:%d/%s"

class PwmBeagle
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::vector<FILE *> ehrDuty_;
  std::vector<FILE *> ehrPeriod_;
  std::vector<FILE *> ehrPolarity_;
  std::vector<FILE *> ehrRun_;
  std::vector<FILE *> ehrRequest_;

  ros::Subscriber pwm_out_sub_;
  ros::AsyncSpinner spinner_;
  
  void openEhrFiles(void);
  void setEhrFreqs(void);
  void closeEhrFiles(void);

public:
  PwmBeagle(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~PwmBeagle(void);
  void setPosition( uint8_t servo_id, int microseconds );
  void pwmOutCallback( falkor_msgs::Pwm::ConstPtr out_msg);
};

#endif /* FALKOR_DRIVERS_PWM_BEAGLE_H */

