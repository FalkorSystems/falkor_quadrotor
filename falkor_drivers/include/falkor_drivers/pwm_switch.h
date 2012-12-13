#ifndef FALKOR_DRIVERS_PWM_SWITCH_H
#define FALKOR_DRIVERS_PWM_SWITCH_H

#include "ros/ros.h"
#include "falkor_msgs/Pwm.h"

class PwmSwitch {
private:
  falkor_msgs::Pwm::ConstPtr pwm_in_;
  falkor_msgs::Pwm::ConstPtr pwm_cmd_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pwm_out_pub_;
  ros::Subscriber pwm_in_sub_;
  ros::Subscriber pwm_cmd_sub_;
  int chan_5_fix_;
  ros::Timer timer_;

public:
  PwmSwitch(ros::NodeHandle nh, ros::NodeHandle nh_private);
  void pwmInCallback(falkor_msgs::Pwm::ConstPtr in_msg);
  void pwmCmdCallback( const falkor_msgs::Pwm::ConstPtr in_msg);
  void loopOnce(const ros::TimerEvent &event);
};

#endif /* FALKOR_DRIVERS_PWM_SWITCH_H */
