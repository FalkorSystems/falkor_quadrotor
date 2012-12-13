#include "falkor_drivers/pwm_switch.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "pwm_switch");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  PwmSwitch pwm_switch(nh, nh_private);
  ros::spin();
  return 0;
}
